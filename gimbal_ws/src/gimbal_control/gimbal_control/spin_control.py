import rclpy
from rclpy.node import Node
import can
#from std_msgs.msg import Float64
from std_msgs.msg import Int32

class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')

        # 初始化CAN接口
        self.bus = can.interface.Bus(channel='can0', interface='socketcan')

        # 记录上机位和下机位的最新角度
        self.last_up_angle = None
        self.last_down_angle = None

        # 创建两个话题订阅者来分别控制上下机位
        self.subscription_up = self.create_subscription(
            #Float64,
            Int32,
            'gimbal_up_position',
            self.move_up_gimbal,
            10
        )
        self.subscription_down = self.create_subscription(
            #Float64,
            Int32,
            'gimbal_down_position',
            self.move_down_gimbal,
            10
        )

    def move_gimbal(self, motor_id, angle):
        # 限制角度范围
        #if angle < -180 or angle > 180:
        if angle < -75 or angle > 165:
            self.get_logger().warning(f'Angle {angle}° is out of range, ignoring command.')
            return

        # 将角度转换为协议格式的值
        angle_value = int((angle + 180) * (65536 / 360) - 32768)
        angle_value = max(-32768, min(32767, angle_value))  # 限制范围
        angle_bytes = angle_value.to_bytes(2, byteorder='big', signed=True)

        # 组装CAN数据帧
        data = bytearray(8)
        data[0] = 0x10  # 移动指令标志位
        data[1:3] = angle_bytes  # 目标角度
        data[3:8] = b'\x00\x00\x00\x00\x00'  # 保留位

        # 发送扩展帧
        msg = can.Message(arbitration_id=motor_id,
                          data=data,
                          is_extended_id=True)
        try:
            self.bus.send(msg)
            self.get_logger().info(f'Sent command to motor {hex(motor_id)}: Angle {angle}°')
            self.log_data_frame(data,motor_id)  # 输出发送的数据帧格式
        except can.CanError as e:
            self.get_logger().error(f'Failed to send command: {e}')

    def log_data_frame(self, data, motor_id):
        hex_data = ' '.join(f'{byte:02X}' for byte in data)
        self.get_logger().info(f'Data frame to motor {hex(motor_id)}: {hex_data}')

    def move_up_gimbal(self, msg):
        angle = msg.data
        # 仅在角度变化时才发送命令
        if angle != self.last_up_angle:
            self.move_gimbal(0x1314, angle)  # 横摇的 CAN ID 为 0x1314
            self.last_up_angle = angle  # 更新最后发送的角度

    def move_down_gimbal(self, msg):
        angle = msg.data - 45
        # 仅在角度变化时才发送命令
        if angle != self.last_down_angle:
            self.move_gimbal(0x1313, angle)  # 俯摇的 CAN ID 为 0x1313
            self.last_down_angle = angle  # 更新最后发送的角度

    def close_can_bus(self):
        # 确保关闭 CAN 总线
        self.bus.shutdown()
        self.get_logger().info("CAN bus has been properly shut down.")

def main(args=None):
    rclpy.init(args=args)
    gimbal_controller = GimbalController()
    try:
        rclpy.spin(gimbal_controller)
    except KeyboardInterrupt:
        print('Shutting down Gimbal Controller node.')
        gimbal_controller.close_can_bus()
    except Exception as e:
        print(f'An error occurred: {e}')
    finally:
        # 如果你需要在这里做其他清理工作，可以在这里做
        pass

if __name__ == '__main__':
    main()