import threading
import time

from xbot_common_interfaces.action import SimpleTrajectory
from xbot_common_interfaces.msg import HybridJointCommand
from sensor_msgs.msg import JointState,Imu

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import math
from rclpy.executors import ExternalShutdownException
import sys


class MinimalActionServer(Node):

    def __init__(self):
        super().__init__('trajectory_server',parameter_overrides=[])
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            SimpleTrajectory,
            'simple_trajectory_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        self._state_lock = threading.Lock()
        self.joint_states = None
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_cb,
            10)
        self.subscription  # prevent unused variable warning

        self.imu_sub = self.create_subscription(
        Imu,
        '/imu_broadcaster/imu',
        self.imu_cb,
        10)

        # Declare parameters and get their values
        self.declare_parameters(
            namespace='',
            parameters=[('tolerance', rclpy.Parameter.Type.DOUBLE),
                        ('joint_names', rclpy.Parameter.Type.STRING_ARRAY),
                        ("duration",rclpy.Parameter.Type.DOUBLE),
                        ("kp",rclpy.Parameter.Type.DOUBLE_ARRAY),
                        ("kd",rclpy.Parameter.Type.DOUBLE_ARRAY)
                       ]
            )
        self.declare_parameter("controller_topic_name", "hybrid_body_controller/commands")
        
        self.tolerance = self.get_parameter('tolerance').value
        self.joint_names = self.get_parameter('joint_names').value
        self.duration = 5.0
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.amplitude = 0.04
        self.frequency = 0.1
        self.controller_topic_name = self.get_parameter('controller_topic_name').value
        self.hybrid_cmd_pub_ = self.create_publisher(HybridJointCommand, f'{self.controller_topic_name}', 1)
        self.current_positions = {}  
        self.target_positions = {name: 0.0 for name in self.joint_names}   

    def imu_cb(self,msg):
        #TODO: implement imu callback
        pass

    def joint_states_cb(self,msg):
        with self._state_lock:
            self.joint_states = msg

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        if goal_request.traj_type == SimpleTrajectory.Goal.SIN_WAVE:
            with self._state_lock:
                joint_positions = dict(zip(self.joint_states.name, self.joint_states.position))
                all_close_to_zero = True
                for name in self.joint_names:
                    if name in joint_positions:
                        if abs(joint_positions[name]) >= self.tolerance:
                            self.get_logger().info(f'Joint {name} position {joint_positions[name]} is not close to zero.')
                            all_close_to_zero = False
        
            if not all_close_to_zero:
                self.get_logger().info('Some specified joint positions are not close to zero.')
                return GoalResponse.REJECT
                
            with self._goal_lock:
                # This server only allows one goal at a time
                if self._goal_handle is not None and self._goal_handle.is_active:
                    self.get_logger().info('Goal rejected because an existing goal is already being processed')
                    return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            self._goal_handle = goal_handle
        self.duration = goal_handle.request.duration
        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def set_joint_positions(self, positions):
        self.current_positions = positions
        cmd = HybridJointCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_name = self.joint_names
        for i in range(len(self.joint_names)):
            cmd.position.append(positions[self.joint_names[i]])
        cmd.velocity = [0.0] * len(self.joint_names)
        cmd.feedforward = [0.0] * len(self.joint_names)
        cmd.kp = self.kp
        cmd.kd = self.kd
        self.hybrid_cmd_pub_.publish(cmd)
        
        #self.get_logger().info(f'Setting joint positions: {positions}')

    def get_joint_positions(self):
        with self._state_lock:
            for name, position in zip( self.joint_states.name,  self.joint_states.position):
                if name in self.joint_names:
                    self.current_positions[name] = position
        return self.current_positions

    def linear_interpolation(self, start_pos, end_pos, start_time, end_time, current_time):

        if current_time >= end_time:
            return end_pos
        elif current_time <= start_time:
            return start_pos
        else:
            return start_pos + (end_pos - start_pos) * (current_time - start_time) / (end_time - start_time)

    def move_to_zero(self,goal_handle):
        start_positions = self.get_joint_positions()
        end_positions = self.target_positions.copy()
        start_time = time.time()
        end_time = start_time + self.duration
        fb = SimpleTrajectory.Feedback()
        while time.time() < end_time:
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return SimpleTrajectory.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return SimpleTrajectory.Result()
            current_time = time.time()
            new_positions = {}
            for joint in self.joint_names:
                new_positions[joint] = self.linear_interpolation(start_positions[joint], end_positions[joint], start_time, end_time, current_time)
            self.set_joint_positions(new_positions)
            fb.progress = (current_time - start_time) / self.duration
            goal_handle.publish_feedback(fb)
            time.sleep(0.01)  

        self.set_joint_positions(end_positions)
        goal_handle.succeed()

    def sin_wave(self, goal_handle):
        start_positions = self.get_joint_positions()
        start_time = time.time()
        end_time = start_time + self.duration
        fb = SimpleTrajectory.Feedback()
        while time.time() < end_time:
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return SimpleTrajectory.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return SimpleTrajectory.Result()
            current_time = time.time()
            new_positions = {}
            for joint in self.joint_names:
                new_positions[joint] = start_positions[joint] + self.amplitude * math.sin(2 * math.pi * self.frequency * (current_time - start_time))
            self.set_joint_positions(new_positions)
            fb.progress = (current_time - start_time) / self.duration
            goal_handle.publish_feedback(fb)
            time.sleep(0.01)  
        goal_handle.succeed()   
        

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        if goal_handle.request.traj_type == SimpleTrajectory.Goal.SIN_WAVE:
            self.sin_wave(goal_handle)
        elif goal_handle.request.traj_type == SimpleTrajectory.Goal.ZERO:
            self.move_to_zero(goal_handle)
                          
        # Populate result message
        result = SimpleTrajectory.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = MinimalActionServer()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(action_server, executor=executor)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        action_server.destroy()
        if rclpy.ok():
            rclpy.shutdown()
    



if __name__ == '__main__':
    main()
