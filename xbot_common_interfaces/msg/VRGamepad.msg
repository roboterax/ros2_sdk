TimingHeader header             # 消息头，包含时间戳和帧ID
geometry_msgs/Pose pose        # 位姿（位置 + 四元数）
float32[] buttons              # 按钮状态（0-1）
float32[] axes                # 摇杆/扳机值（-1.0到1.0）