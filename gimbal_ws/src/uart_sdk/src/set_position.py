import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # 串口初始化
        self.ser.flush()

        # 创建一个订阅者来监听控制命令
        self.subscription = self.create_subscription(
            String,
            'gimbal_control_topic',
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用变量的警告

    def listener_callback(self, msg):
        # 假设收到的消息格式为 "servo_id position"
        try:
            servo_id, position = map(int, msg.data.split())
            self.set_position(servo_id, position)
        except ValueError:
            self.get_logger().warn("Invalid message format, expected 'servo_id position'.")

    def set_position(self, servo_id, position):
        # 计算并发送控制指令给舵机
        # 这里使用协议格式，发送位置指令给指定的舵机
        header = b'\xFF\xFF'
        length = 0x04  # 数据长度
        instruction_type = 0x03  # 写数据指令
        address = 0x2A  # 控制目标位置的地址
        position_bytes = position.to_bytes(2, 'big')  # 目标位置（大端字节序）
        
        # 计算校验和
        checksum = ~(servo_id + length + instruction_type + address + position_bytes[0] + position_bytes[1]) & 0xFF
        
        # 构造指令
        command = header + bytes([servo_id]) + bytes([length]) + bytes([instruction_type]) + bytes([address]) + position_bytes + bytes([checksum])
        
        # 发送指令
        self.ser.write(command)
        self.ser.flush()
        self.get_logger().info(f'Sent position {position} to servo {servo_id}.')

def main(args=None):
    rclpy.init(args=args)
    gimbal_controller = GimbalController()
    rclpy.spin(gimbal_controller)

    gimbal_controller.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
