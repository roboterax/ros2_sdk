# Parallel interface to ServoPose (/servo_poses).
# Field order is the same: left, right, head — each is a timed cartesian trajectory.
#
# Topic: /servo_pose_trajectories
# Intended for inference / policy rollout. Teleop keeps using /servo_poses.
#
# header.stamp is message publish time (optional diagnostics), not the trajectory clock.
# Waypoint timing is time_from_start on each point, in seconds, relative to the trajectory start.
std_msgs/Header header
ServoPoseTrajectory left_pose
ServoPoseTrajectory right_pose
ServoPoseTrajectory head_pose
