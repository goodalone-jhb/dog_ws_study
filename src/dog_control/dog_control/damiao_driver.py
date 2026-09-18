import math
import struct

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# ===== 达妙 MIT 模式参数范围（DM4310）=====
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -30.0, 30.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -32.0, 32.0

KP = 15.0
KD = 1.6

# ===== 关节顺序约定 =====
JOINT_NAMES = [
    'FL_hip_joint',   'FL_thigh_joint', 'FL_calf_joint',
    'FR_hip_joint',   'FR_thigh_joint', 'FR_calf_joint',
    'RL_hip_joint',   'RL_thigh_joint', 'RL_calf_joint',
    'RR_hip_joint',   'RR_thigh_joint', 'RR_calf_joint',
]

# ===== 关节顺序：FL(0,1,2) FR(3,4,5) RL(6,7,8) RR(9,10,11) =====
LEG_INDEX = {
    'RL': [6, 7, 8],
    'RR': [9, 10, 11],
}

# ===== 接了哪几条腿：每条腿各占一条 CAN 总线 =====
# 每条总线都是独立的一网，所以三台电机的 ID 都是 1/2/3 也不冲突
LEGS = [
    {'name': 'RL', 'channel': 'can2', 'indices': LEG_INDEX['RL']},
    {'name': 'RR', 'channel': 'can1', 'indices': LEG_INDEX['RR']},
]
MOTOR_IDS = [1, 2, 3]


def float_to_uint(x, x_min, x_max, bits):
    x = max(min(x, x_max), x_min)
    return int((x - x_min) * ((1 << bits) - 1) / (x_max - x_min))


# ===== 反馈帧（电机 → 本机）=====
# 电机每 20 ms 主动回一帧：CAN ID = 0x00，6 字节
#   D0     = 电机 ID
#   D1 D2  = 位置 16 bit（量程同 P_MIN..P_MAX）
#   D3     = 速度 12 位里的高 8 位
#   D4 高4 = 速度 12 位里的低 4 位 ; D4 低4 = 力矩 12 位里的高 4 位
#   D5     = 力矩 12 位里的低 8 位
FEEDBACK_CAN_ID = 0x00


def uint_to_float(x, x_min, x_max, bits):
    """float_to_uint 的反运算：把整数位模式还原成物理量。"""
    return float(x) * (x_max - x_min) / ((1 << bits) - 1) + x_min


def decode_feedback(data):
    """把 6 字节反馈解析成 (位置 rad, 速度 rad/s, 力矩 N·m)。"""
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | (data[4] >> 4)
    tau_int = ((data[4] & 0x0F) << 8) | data[5]

    pos = uint_to_float(pos_int, P_MIN, P_MAX, 16)
    vel = uint_to_float(vel_int, V_MIN, V_MAX, 12)
    tau = uint_to_float(tau_int, T_MIN, T_MAX, 12)
    return pos, vel, tau


def pack_cmd(pos, vel, kp, kd, torque):
    p = float_to_uint(pos, P_MIN, P_MAX, 16)
    v = float_to_uint(vel, V_MIN, V_MAX, 12)
    kp_i = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_i = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    t = float_to_uint(torque, T_MIN, T_MAX, 12)

    return [
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp_i >> 8) & 0xF),
        kp_i & 0xFF,
        (kd_i >> 4) & 0xFF,
        ((kd_i & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF,
    ]


def send_special(bus, motor_id, cmd):
    bus.send(can.Message(
        arbitration_id=motor_id,
        data=[0xFF] * 7 + [cmd],
        is_extended_id=False
    ))


class FeedbackListener(can.Listener):
    """CAN 收帧回调：只解析，把结果写进 leg.state。

    Notifier 在它自己的线程里调用 on_message_received，
    所以这里绝不碰 ROS 发布器；ROS 发送统一在 publish_state 定时器里做。
    """

    def __init__(self, leg):
        self.leg = leg

    def on_message_received(self, msg):
        # 只要电机的反馈帧：ID=0x00 且 6 字节。
        # ID=1/2/3 的 8 字节帧是本机自己发出去的指令，必须排除掉
        if msg.arbitration_id != FEEDBACK_CAN_ID or len(msg.data) != 6:
            return
        motor_id = msg.data[0]
        if motor_id not in MOTOR_IDS:
            return
        self.leg.state[motor_id] = decode_feedback(msg.data)


class Leg:
    """一条腿 = 一条 CAN 总线 + 3 台电机 + 3 个关节索引 + 自己的反馈缓存。

    把"每条腿各自的东西"都收在这里，节点就只用遍历 LEGS，不用到处写死。
    """

    def __init__(self, name, channel, indices):
        self.name = name
        self.indices = indices                            # 本腿在 /joint_command 里的 3 个索引
        self.bus = can.interface.Bus(channel=channel, interface='socketcan')
        self.state = {}                                   # motor_id -> (位置, 速度, 力矩)
        self.notifier = None                              # 收帧线程，由节点创建

    def enable(self):
        for mid in MOTOR_IDS:
            send_special(self.bus, mid, 0xFC)

    def disable(self):
        for mid in MOTOR_IDS:
            try:
                send_special(self.bus, mid, 0xFD)
            except Exception:
                pass

    def send_command(self, data):
        """data 是完整的 12 个关节目标；本腿只取自己那 3 个，发给自己的 3 台电机。"""
        for mid, idx in zip(MOTOR_IDS, self.indices):
            self.bus.send(can.Message(
                arbitration_id=mid,
                data=pack_cmd(data[idx], 0.0, KP, KD, 0.0),
                is_extended_id=False
            ))


class DamiaoDriver(Node):
    def __init__(self):
        super().__init__('damiao_driver')

        # ===== 每条腿：开自己的总线 → 起自己的收帧线程 → 使能自己的 3 台电机 =====
        self.legs = []
        for cfg in LEGS:
            leg = Leg(cfg['name'], cfg['channel'], cfg['indices'])
            leg.notifier = can.Notifier(leg.bus, [FeedbackListener(leg)])
            leg.enable()
            self.legs.append(leg)
            self.get_logger().info(f"{leg.name} 腿：{cfg['channel']} 已连接并使能")

        # ===== 两条腿的反馈合起来发到 /joint_states（闭环要用它，RViz 也用它）=====
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.state_timer = self.create_timer(0.02, self.publish_state)   # 50 Hz

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/joint_command',
            self.command_callback,
            10
        )

    def command_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 12:
            self.get_logger().warn(f'收到 {len(msg.data)} 个数据，期望 12 个')
            return

        for leg in self.legs:
            leg.send_command(msg.data)

    def publish_state(self):
        """把两条腿最近收到的反馈合成一帧 JointState（没收到反馈的关节就不发）。"""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for leg in self.legs:
            for motor_id, idx in zip(MOTOR_IDS, leg.indices):
                if motor_id not in leg.state:   # 还没收到这台的反馈，这轮跳过它
                    continue
                pos, vel, tau = leg.state[motor_id]
                js.name.append(JOINT_NAMES[idx])
                js.position.append(pos)
                js.velocity.append(vel)
                js.effort.append(tau)

        if js.name:                             # 一台都没收到就不发空消息
            self.state_pub.publish(js)

    def shutdown(self):
        for leg in self.legs:
            try:
                leg.notifier.stop()             # 先停收帧线程
            except Exception:
                pass
            leg.disable()                        # 再失能
            try:
                leg.bus.shutdown()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = DamiaoDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
