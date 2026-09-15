import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # 发布 12 个关节的目标位置
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/joint_command',
            10
        )

        # 10 Hz 定时器
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.t = 0.0
        self.get_logger().info('joint_command_publisher 已启动')

    def timer_callback(self):
        msg = Float64MultiArray()

        # 先发 12 个假数据，每个关节做小幅正弦摆动
        # 顺序先约定为：FL_hip, FL_thigh, FL_calf,
        #               FR_hip, FR_thigh, FR_calf,
        #               RL_hip, RL_thigh, RL_calf,
        #               RR_hip, RR_thigh, RR_calf
        msg.data = [
            0.3 * math.sin(self.t + i * math.pi / 6.0)
            for i in range(12)
        ]

        self.publisher_.publish(msg)
        self.get_logger().info(f'发布: {[round(x, 3) for x in msg.data]}')

        self.t += 0.1


def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
