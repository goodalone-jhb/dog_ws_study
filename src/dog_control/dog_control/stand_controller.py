import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'FL_hip_joint',   'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint',   'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint',   'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint',   'RR_thigh_joint', 'RR_calf_joint',
]

# 站立姿态：hip=0, thigh=0.4, calf=-0.8
STAND_POSE = []
for _ in range(4):
    STAND_POSE += [0.0, 0.4, -0.8]


class StandController(Node):
    def __init__(self):
        super().__init__('stand_controller')

        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # 启动后 1 秒发一次轨迹
        self.timer = self.create_timer(1.0, self.send_trajectory)
        self.sent = False

        self.get_logger().info('stand_controller 已启动')

    def send_trajectory(self):
        if self.sent:
            return

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = STAND_POSE
        point.velocities = [0.0] * 12
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points = [point]
        self.pub.publish(msg)

        self.get_logger().info('已发送站立轨迹')
        self.sent = True


def main(args=None):
    rclpy.init(args=args)
    node = StandController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()