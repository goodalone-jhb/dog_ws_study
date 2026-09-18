import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'FL_hip_joint',   'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint',   'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint',   'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint',   'RR_thigh_joint', 'RR_calf_joint',
]


class JointCommandBridge(Node):
    def __init__(self):
        super().__init__('joint_command_bridge')

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_command',
            self.command_callback,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.get_logger().info('joint_command_bridge 已启动')

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warn(
                f'收到 {len(msg.data)} 个数据，期望 {len(JOINT_NAMES)} 个，已忽略'
            )
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = list(msg.data)
        js.velocity = [0.0] * len(JOINT_NAMES)
        js.effort = [0.0] * len(JOINT_NAMES)

        self.pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()