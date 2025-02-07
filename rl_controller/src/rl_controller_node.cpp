#include <ATen/Parallel.h>
#include <sys/mman.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rl_controller/rl_controller.hpp"

// Using only the necessary namespace members.
using namespace std::chrono_literals;

RLControllerNode::RLControllerNode()
    : Node("rl_controller_node"),
      pos_des{{0.0}},
      vel_des{{0.0}},
      kp{{0.0}},
      kd{{0.0}},
      torque{{0.0}},
      initial_pos{{0.0}} {
  // Use a QoS with a queue size of 1.
  rclcpp::QoS qos_settings =
      rclcpp::QoS(1)
          .reliability(rclcpp::ReliabilityPolicy::Reliable)
          .durability(rclcpp::DurabilityPolicy::Volatile);

  imu_sub_ = this->create_subscription<xbot_common_interfaces::msg::Imu>(
      "/imu_feedback", qos_settings,
      std::bind(&RLControllerNode::ImuCallback, this, std::placeholders::_1));

  command_sub_ =
      this->create_subscription<xbot_common_interfaces::msg::ChannelsMsg>(
          "/loco/remoteControl/radio", qos_settings,
          std::bind(&RLControllerNode::CommandCallback, this,
                    std::placeholders::_1));

  motor_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/motor_feedback", qos_settings,
      std::bind(&RLControllerNode::MotorCallback, this, std::placeholders::_1));

  policy_inference_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/rl_controller/policy_inference", qos_settings);

  this->declare_parameter("policy_file", policy_file_);
  this->get_parameter("policy_file", policy_file_);
  if (policy_file_.empty()) {
    policy_file_ = "policy_walk_open3.pt";
    RCLCPP_WARN(this->get_logger(),
                "Policy file not specified, using default: %s",
                policy_file_.c_str());
  }
  inference_thread_ = std::thread(&RLControllerNode::InferenceLoop, this);

  // Lock memory to prevent paging (for real-time performance).
  mlockall(MCL_CURRENT | MCL_FUTURE);
}

RLControllerNode::~RLControllerNode() {
  if (inference_thread_.joinable()) {
    inference_thread_.join();
  }
}

void RLControllerNode::Initialize() {
  count_ = 0;
  // Initialize single observation tensor (size 47).
  tmp_obs = torch::zeros({N_SINGLE_POLICY_OBS});

  // Initialize observation history with N_HIST_LEN zero tensors.
  obs_history.clear();
  for (int i = 0; i < N_HIST_LEN; i++) {
    obs_history.push_back(torch::zeros({N_SINGLE_POLICY_OBS}));
  }

  // Initialize stacked observation tensor (size 47 * N_HIST_LEN).
  stacked_obs = torch::zeros({N_POLICY_OBS});

  // Initialize input tensor for the model.
  tensor.clear();
  tensor.push_back(torch::zeros({N_POLICY_OBS}));

  // Load the TorchScript policy model.
  try {
    std::string model_path =
        ament_index_cpp::get_package_share_directory("rl_controller") +
        "/model/" + policy_file_;
    policy = torch::jit::load(model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Successfully loaded policy model from: %s",
                model_path.c_str());
  } catch (const c10::Error &e) {
    RCLCPP_ERROR(this->get_logger(), "Error loading policy model: %s",
                 e.what());
    throw;
  }

  // Test a forward pass.
  tensor[0] = torch::zeros({N_POLICY_OBS}).unsqueeze(0);
  try {
    auto output = policy.forward(tensor);
    RCLCPP_INFO(this->get_logger(), "Policy model initialization successful");
  } catch (const c10::Error &e) {
    RCLCPP_ERROR(this->get_logger(), "Error during policy forward pass: %s",
                 e.what());
    throw;
  }
}

void RLControllerNode::CommandCallback(
    const xbot_common_interfaces::msg::ChannelsMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  command_data_ = *msg;
  command_data_received_ = true;
}

void RLControllerNode::ImuCallback(
    const xbot_common_interfaces::msg::Imu::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(imu_mutex_);
  imu_data_ = *msg;
  imu_data_received_ = true;
}

void RLControllerNode::MotorCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(motor_mutex_);
  motor_data_ = *msg;
  motor_data_received_ = true;
}

void RLControllerNode::InferenceLoop() {
  rclcpp::Rate rate(CONTROL_FREQUENCY);
  int step_count = 0;
  std::chrono::duration<double, std::milli> cumulative_duration(0);

  while (rclcpp::ok()) {
    {
      auto start = std::chrono::high_resolution_clock::now();

      // Make local copies of the data with locks
      xbot_common_interfaces::msg::Imu local_imu_data;
      xbot_common_interfaces::msg::ChannelsMsg local_command_data;
      std_msgs::msg::Float64MultiArray local_motor_data;
      bool data_valid = true;

      {
        std::lock_guard<std::mutex> imu_lock(imu_mutex_);
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        std::lock_guard<std::mutex> motor_lock(motor_mutex_);

        // Check if we have received all necessary data
        if (!imu_data_received_ || !command_data_received_ ||
            !motor_data_received_) {
          data_valid = false;
        } else {
          local_imu_data = imu_data_;
          local_command_data = command_data_;
          local_motor_data = motor_data_;
        }
      }

      if (!data_valid) {
        // Data not yet received; reset count_ and skip this iteration
        count_ = 0;
        ResetControllerState();
        rate.sleep();
        continue;
      }

      // Validate motor data size
      if (local_motor_data.data.size() <
          N_JOINTS * 2) {  // We need position and velocity data
        RCLCPP_WARN(this->get_logger(), "Incomplete motor data received");
        rate.sleep();
        continue;
      }

      Perform_inference(local_imu_data, local_command_data, local_motor_data);

      // Monitor performance
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration<double, std::milli>(end - start);
      cumulative_duration += duration;

      // Print the average inference FPS only when RL has taken control.
      step_count++;
      float move_safe = (local_command_data.channels[9] > 0.1f) ? 0.0f : 1.0f;

      if (move_safe < 0.1f) {
        step_count = 0;
      }
      if ((step_count % 100 == 0) && (step_count > RUN_TIME)) {
        double avg_duration = cumulative_duration.count() / 100;
        RCLCPP_INFO(this->get_logger(), "Average inference FPS: %.1f",
                    1000.0 / avg_duration);
        cumulative_duration = std::chrono::duration<double, std::milli>(0);
      }
    }

    rate.sleep();
  }
}

void RLControllerNode::Perform_inference(
    const xbot_common_interfaces::msg::Imu &imu_data,
    const xbot_common_interfaces::msg::ChannelsMsg &command_data,
    const std_msgs::msg::Float64MultiArray &motor_data) {
  count_++;
  float move_safe = (command_data.channels[9] > 0.1f) ? 0.0f : 1.0f;

  if (move_safe < 0.1f) {
    count_ = 1;
    ResetControllerState();
  }

  // Stage logging only when count_ < RUN_TIME; when count_ equals RUN_TIME,
  // print "RL take control"
  if (count_ < RUN_TIME) {
    if (count_ % 100 ==
        0) {  // Log every 100 steps while still in the pre-RL phase.
      if (count_ < INITIAL_TIME) {
        RCLCPP_INFO(this->get_logger(), "Stage: Initialization (%d/%d)", count_,
                    INITIAL_TIME);
      } else if (count_ <= STARTUP_TIME) {
        RCLCPP_INFO(this->get_logger(),
                    "Stage: Moving to default positions (%d/%d)",
                    count_ - INITIAL_TIME, STARTUP_TIME - INITIAL_TIME);
      } else {
        RCLCPP_INFO(this->get_logger(), "Stage: Standing still (%d/%d)",
                    count_ - STARTUP_TIME, RUN_TIME - STARTUP_TIME);
      }
    }
  } else if (count_ == RUN_TIME) {
    RCLCPP_INFO(this->get_logger(), "RL take control");
  }

  if (count_ < RUN_TIME) {
    // [0, INITIAL_TIME]: Initialize and store initial joint positions.
    if (count_ < INITIAL_TIME) {
      pos_des.fill(0.0);
      kp.fill(0.0);
      kd.fill(50.0);
      std::copy_n(motor_data.data.begin(), N_JOINTS, initial_pos.begin());
    }
    // [INITIAL_TIME, STARTUP_TIME]: Interpolate to default positions.
    else if (count_ <= STARTUP_TIME) {
      std::fill(kp.begin(), kp.end(), 400.0);
      std::fill(kd.begin(), kd.end(), 10.0);

      double ratio =
          std::clamp(static_cast<double>(count_ - INITIAL_TIME) /
                         static_cast<double>(STARTUP_TIME - INITIAL_TIME),
                     0.0, 1.0);

      for (int i = 0; i < N_JOINTS; i++) {
        pos_des[i] =
            initial_pos[i] + ratio * (default_joint_pos[i] - initial_pos[i]);
      }
    }
    // [STARTUP_TIME, RUN_TIME]: Hold the default positions.
    else {
      std::fill(kp.begin(), kp.end(), 400.0);
      std::fill(kd.begin(), kd.end(), 10.0);
      for (size_t i = 0; i < pos_des.size(); ++i) {
        pos_des[i] = static_cast<double>(default_joint_pos[i]);
      }
    }

    SendToMotor(pos_des, vel_des, kp, kd, torque);

    return;
  }

  static std::array<double, N_CONTROLLED_JOINTS> last_action{};

  // Calculate the norm of selected command channels.
  float norm = std::hypot(command_data.channels[2], command_data.channels[3],
                          command_data.channels[0]);

  // Determine whether to move.
  float move = (norm < 0.1) ? 0.0f : 1.0f;
  float move_RC = (command_data.channels[8] > 0.1f) ? 0.0f : 1.0f;
  move *= (move_RC * move_safe);

  curr_obs_ind = 0;

  // Fill tmp_obs with current observation:
  // 1. Periodic signals.
  tmp_obs[curr_obs_ind++] =
      sin(2 * M_PI * count_ * (1.0f / CONTROL_FREQUENCY) / CYCLE_TIME);
  tmp_obs[curr_obs_ind++] =
      cos(2 * M_PI * count_ * (1.0f / CONTROL_FREQUENCY) / CYCLE_TIME);

  // 2. Command signals.
  speed_x = command_data.channels[2];
  speed_x = (speed_x > 0) ? 1.2f * speed_x : 0.4f * speed_x;
  tmp_obs[curr_obs_ind++] = move * speed_x;
  tmp_obs[curr_obs_ind++] = move * (-0.4f * command_data.channels[3]);
  tmp_obs[curr_obs_ind++] = move * (-0.4f * command_data.channels[0]);

  // 3. Joint positions difference from default (N_CONTROLLED_JOINTS values).
  for (int i = 0; i < N_CONTROLLED_JOINTS; i++) {
    tmp_obs[curr_obs_ind++] = motor_data.data[N_UNCONTROLLED_JOINTS + i] -
                              default_joint_pos[N_UNCONTROLLED_JOINTS + i];
  }

  // 4. Scaled joint velocities (N_CONTROLLED_JOINTS values).
  for (int i = 0; i < N_CONTROLLED_JOINTS; i++) {
    tmp_obs[curr_obs_ind++] =
        0.1f * motor_data.data[N_JOINTS + N_UNCONTROLLED_JOINTS + i];
  }

  // 5. Previous action (N_CONTROLLED_JOINTS values).
  for (int i = 0; i < N_CONTROLLED_JOINTS; i++) {
    tmp_obs[curr_obs_ind++] = last_action[i];
  }

  // 6. Base angular velocity (3 values).
  tmp_obs[curr_obs_ind++] = imu_data.angular_vel_x;
  tmp_obs[curr_obs_ind++] = imu_data.angular_vel_y;
  tmp_obs[curr_obs_ind++] = imu_data.angular_vel_z;

  // 7. Base Euler angles (3 values).
  tmp_obs[curr_obs_ind++] = imu_data.roll;
  tmp_obs[curr_obs_ind++] = imu_data.pitch;
  tmp_obs[curr_obs_ind++] = imu_data.yaw;
  // curr_obs_ind should now equal N_SINGLE_POLICY_OBS (47).

  // Update observation history:
  obs_history.pop_front();                 // Remove the oldest observation.
  obs_history.push_back(tmp_obs.clone());  // Append the latest observation.

  // Stack the observations into a single tensor.
  int offset = 0;
  for (const auto &obs : obs_history) {
    stacked_obs.slice(0, offset, offset + N_SINGLE_POLICY_OBS) = obs;
    offset += N_SINGLE_POLICY_OBS;
  }

  // Run inference with the stacked observation.
  tensor[0] = stacked_obs.unsqueeze(0).clamp(MIN_CLIP, MAX_CLIP);
  auto output = policy.forward(tensor);
  out = output.toTensor().index({0}).clamp(MIN_CLIP, MAX_CLIP);

  // Low-pass filter the actions.
  for (int i = 0; i < N_CONTROLLED_JOINTS; i++) {
    double clipped_action = out.index({i}).item<float>();
    last_action[i] = 0.02 * last_action[i] + 0.98 * clipped_action;
  }

  // Set default gains and positions for joints 0-18.
  for (int i = 0; i < N_UNCONTROLLED_JOINTS; i++) {
    kp[i] = 300.0;
    kd[i] = 50.0;
    pos_des[i] = 0.0;
    vel_des[i] = 0.0;
    torque[i] = 0.0;
  }

  // Hip roll
  for (int i : {0, 6}) {
    kp[N_UNCONTROLLED_JOINTS + i] = 250;
    kd[N_UNCONTROLLED_JOINTS + i] = 20.0;
    pos_des[N_UNCONTROLLED_JOINTS + i] =
        0.25 * last_action[i] + default_joint_pos[N_UNCONTROLLED_JOINTS + i];
    vel_des[N_UNCONTROLLED_JOINTS + i] = 0.0;
    torque[N_UNCONTROLLED_JOINTS + i] = 0.0;
  }

  // Hip yaw
  for (int i : {1, 7}) {
    kp[N_UNCONTROLLED_JOINTS + i] = 200.0;
    kd[N_UNCONTROLLED_JOINTS + i] = 10.0;
    pos_des[N_UNCONTROLLED_JOINTS + i] =
        0.25 * last_action[i] + default_joint_pos[N_UNCONTROLLED_JOINTS + i];
    vel_des[N_UNCONTROLLED_JOINTS + i] = 0.0;
    torque[N_UNCONTROLLED_JOINTS + i] = 0.0;
  }

  // Pitch & knee
  for (int i : {2, 3, 8, 9}) {
    kp[N_UNCONTROLLED_JOINTS + i] = 400.0;
    kd[N_UNCONTROLLED_JOINTS + i] = 20.0;
    pos_des[N_UNCONTROLLED_JOINTS + i] =
        0.25 * last_action[i] + default_joint_pos[N_UNCONTROLLED_JOINTS + i];
    vel_des[N_UNCONTROLLED_JOINTS + i] = 0.0;
    torque[N_UNCONTROLLED_JOINTS + i] = 0.0;
  }

  // Ankle
  for (int i : {4, 5, 10, 11}) {
    kp[N_UNCONTROLLED_JOINTS + i] = 40.0;
    kd[N_UNCONTROLLED_JOINTS + i] = 10.0;
    pos_des[N_UNCONTROLLED_JOINTS + i] =
        0.25 * last_action[i] + default_joint_pos[N_UNCONTROLLED_JOINTS + i];
    vel_des[N_UNCONTROLLED_JOINTS + i] = 0.0;
    torque[N_UNCONTROLLED_JOINTS + i] = 0.0;
  }

  if (move_safe == 0.0f) {
    pos_des.fill(0.0);
    vel_des.fill(0.0);
    kp.fill(0.0);
    kd.fill(50.0);
    torque.fill(0.0);
  }

  SendToMotor(pos_des, vel_des, kp, kd, torque);
}

void RLControllerNode::SendToMotor(std::array<double, N_JOINTS> &pos_des,
                                   std::array<double, N_JOINTS> &vel_des,
                                   std::array<double, N_JOINTS> &kp,
                                   std::array<double, N_JOINTS> &kd,
                                   std::array<double, N_JOINTS> &torque) {
  std_msgs::msg::Float64MultiArray msg;
  // Append control commands in order: positions, velocities, kp, kd, torques.
  for (int i = 0; i < N_JOINTS; i++) {
    msg.data.push_back(pos_des[i]);
  }
  for (int i = 0; i < N_JOINTS; i++) {
    msg.data.push_back(vel_des[i]);
  }
  for (int i = 0; i < N_JOINTS; i++) {
    msg.data.push_back(kp[i]);
  }
  for (int i = 0; i < N_JOINTS; i++) {
    msg.data.push_back(kd[i]);
  }
  for (int i = 0; i < N_JOINTS; i++) {
    msg.data.push_back(torque[i]);
  }
  policy_inference_pub_->publish(msg);
}

int main(int argc, char *argv[]) {
  at::set_num_threads(3);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<RLControllerNode>();
  node->Initialize();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
