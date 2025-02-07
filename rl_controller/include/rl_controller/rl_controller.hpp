#ifndef RL_CONTROLLER_HPP
#define RL_CONTROLLER_HPP

#include <ATen/Parallel.h>
#include <fcntl.h>
#include <torch/script.h>
#include <torch/torch.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>

#include "xbot_common_interfaces/msg/channels_msg.hpp"
#include "xbot_common_interfaces/msg/imu.hpp"

// Constants
constexpr double MIN_CLIP = -18.0;
constexpr double MAX_CLIP = 18.0;

constexpr int N_ARM_JOINTS = 14;
constexpr int N_NECT_JOINTS = 2;
constexpr int N_WAIST_JOINTS = 3;
constexpr int N_LEG_JOINTS = 12;
constexpr int N_CONTROLLED_JOINTS = N_LEG_JOINTS;
constexpr int N_UNCONTROLLED_JOINTS =
    N_ARM_JOINTS + N_NECT_JOINTS + N_WAIST_JOINTS;
constexpr int N_JOINTS = N_UNCONTROLLED_JOINTS + N_CONTROLLED_JOINTS;

constexpr int CONTROL_FREQUENCY = 100;
constexpr float CYCLE_TIME = 0.9f;

// Observation stacking parameters.
constexpr int N_SINGLE_POLICY_OBS = 47;
constexpr int N_HIST_LEN = 15;
constexpr int N_POLICY_OBS = N_SINGLE_POLICY_OBS * N_HIST_LEN;

// Timing constants (in control steps)
constexpr int INITIAL_TIME = 100;  // Do nothing; just initialize.
constexpr int STARTUP_TIME = 600;  // Interpolate to default joint positions.
constexpr int RUN_TIME = 1500;     // Stand still.

class RLControllerNode : public rclcpp::Node {
 public:
  RLControllerNode();
  ~RLControllerNode();
  void Initialize();

 private:
  void ImuCallback(const xbot_common_interfaces::msg::Imu::SharedPtr msg);
  void CommandCallback(
      const xbot_common_interfaces::msg::ChannelsMsg::SharedPtr msg);
  void MotorCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void InferenceLoop();
  void Perform_inference(
      const xbot_common_interfaces::msg::Imu &imu_data,
      const xbot_common_interfaces::msg::ChannelsMsg &command_data,
      const std_msgs::msg::Float64MultiArray &motor_data);
  void SendToMotor(std::array<double, N_JOINTS> &pos_des,
                   std::array<double, N_JOINTS> &vel_des,
                   std::array<double, N_JOINTS> &kp,
                   std::array<double, N_JOINTS> &kd,
                   std::array<double, N_JOINTS> &torque);

  // Helper method to reset controller state.
  void ResetControllerState() {
    pos_des.fill(0.0);
    vel_des.fill(0.0);
    kp.fill(0.0);
    kd.fill(0.0);
    torque.fill(0.0);
  }

  // Subscribers
  rclcpp::Subscription<xbot_common_interfaces::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<xbot_common_interfaces::msg::ChannelsMsg>::SharedPtr
      command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr motor_sub_;

  // Publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      policy_inference_pub_;

  // Replace double-buffered data with single instances and mutexes
  xbot_common_interfaces::msg::Imu imu_data_;
  xbot_common_interfaces::msg::ChannelsMsg command_data_;
  std_msgs::msg::Float64MultiArray motor_data_;

  std::mutex imu_mutex_;
  std::mutex command_mutex_;
  std::mutex motor_mutex_;

  bool imu_data_received_{false};
  bool command_data_received_{false};
  bool motor_data_received_{false};

  // Control state.
  std::array<double, N_JOINTS> pos_des;
  std::array<double, N_JOINTS> vel_des;
  std::array<double, N_JOINTS> kp;
  std::array<double, N_JOINTS> kd;
  std::array<double, N_JOINTS> torque;
  std::array<double, N_JOINTS> initial_pos;

  int count_ = 0;
  int curr_obs_ind = 0;
  float ratio = 0.0f;
  float pitch_command = 0.0f;
  float pitch_command_scaled = 0.0f;
  float elbow_command = 0.0f;
  float elbow_command_scaled = 0.0f;
  float speed_x = 0.0f;

  std::thread inference_thread_;

  // Observation tensors and history.
  torch::Tensor tmp_obs;  // shape: {N_SINGLE_POLICY_OBS}
  std::deque<torch::Tensor> obs_history;
  torch::Tensor stacked_obs;  // shape: {N_POLICY_OBS}
  std::vector<torch::jit::IValue> tensor;
  torch::jit::script::Module policy;
  torch::Tensor out;

  std::array<double, 12> previous_command;
  std::array<double, 12> current_command;

  // Default joint positions. (These values will be preserved by NOT
  // reinitializing in the constructor.)
  std::array<float, N_JOINTS> default_joint_pos{
      0, 0, 0, 0, 0, 0,     0,     0,     0, 0, 0, 0,     0,     0,     0, 0,
      0, 0, 0, 0, 0, 0.25f, -0.5f, 0.25f, 0, 0, 0, 0.25f, -0.5f, 0.25f, 0};
  std::string policy_file_;
};

#endif  // RL_CONTROLLER_HPP
