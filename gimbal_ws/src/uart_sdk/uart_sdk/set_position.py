import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time  # 添加time模块导入

class GimbalController(Node):
    def __init__(self):
        super().__init__('gimbal_controller')
        self.ser = None
        self.initialized = False  # 添加初始化标志位
        self.current_vertical_angle = 0  # 添加变量跟踪当前垂直角度

        # 串口初始化
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.ser.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()

        # 创建一个订阅者来监听控制命令
        self.subscription = self.create_subscription(
            String,
            'gimbal_control_topic',
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用变量的警告

        # 在构造函数中进行初始化
        if not self.initialized:
            self.initialize_servos()
            self.initialized = True

    def initialize_servos(self):
        # 初始化舵机位置
        self.get_logger().info("正在初始化舵机...")
        # 先初始化垂直舵机到安全角度
        self.set_position(2, 30, 1000)  # 设置舵机2为30度
        time.sleep(3)  # 等待2.5秒，确保舵机2移动到位
        
        # 初始化水平舵机
        self.set_position(1, 0, 1000)  # 设置舵机1为0度
        time.sleep(3)  # 等待3.5秒，确保舵机1移动到位
        
        # 如果需要，再将垂直舵机调整到0度
        self.set_position(2, 15, 1000)  # 设置舵机2为15度
        time.sleep(2)  # 等待2秒，确保舵机2移动到位

    def listener_callback(self, msg):
        # 假设收到的消息格式为 "servo_id position execution_time"
        try:
            parts = msg.data.split()
            if len(parts) != 3:
                self.get_logger().warn("消息格式无效，预期格式为 'servo_id position execution_time'")
                return
            
            servo_id, position, execution_time = map(int, parts)
            self.set_position(servo_id, position, execution_time)
        except ValueError as e:
            self.get_logger().warn(f"参数转换错误: {e}")
        except Exception as e:
            self.get_logger().error(f"处理消息时发生错误: {e}")

    def angle_to_position(self, angle):
        # 将角度转换为位置值 (0 到 4096)
        angle = int(((angle + 180) / 360) * 4096)
        return angle

    def angle_to_position_l_r(self, angle):
        # 将角度转换为位置值 (0 到 4096)
        if angle >= -120 and angle <= 120:
            angle = int(((angle + 180) / 360) * 4096)
            return angle
        else:
            self.get_logger().error(f"Horizontal angle {angle}° out of range [-120, 120].")
            return None

    def angle_to_position_u_d(self, angle):
        # 将角度转换为位置值 (0 到 4096)
        if angle >=-10 and angle <= 90:
            angle = 90 - angle
            angle = int(((angle + 180) / 360) * 4096)
            return angle
        else:
            self.get_logger().error(f"Horizontal angle {angle}° out of range [-10, 90].")
            return None
    
    def set_position(self, servo_id, angle, execution_time):
        # 如果是水平转动（servo_id == 1），需要检查垂直角度
        if servo_id == 1:
            if self.current_vertical_angle < 15:
                self.get_logger().warn("水平转动需要云台向上角度大于15度！")
                return
            position = self.angle_to_position_l_r(angle)
        elif servo_id == 2:
            #假设 servo_id 2 使用垂直轴的转换函数
            # 更新当前垂直角度
            self.current_vertical_angle = angle
            position = self.angle_to_position_u_d(angle)
        else:
            # 默认使用一般的角度转换函数
            position = self.angle_to_position(angle)

        if position is None:
            # 如果角度转换失败，直接返回
            self.get_logger().error(f"Failed to convert angle for servo {servo_id}.")
            return      

        # 计算并发送控制指令给舵机
        header = b'\xFF\xFF'  # 字头
        # 数据长度：ID号(1) + 指令类型(1) + 参数部分(地址(1) + 目标位置(2) + 执行时间(2)) = 7字节
        length = 7  # 数据长度：舵机 ID(1) + 指令类型(1) + 参数部分(5)
        instruction_type = 0x03  # 写数据指令
        address = 0x2A  # 控制目标位置的地址
        
 
        position_bytes = position.to_bytes(2, 'big')  # 将位置值转为2字节（大端字节序）

        execution_time_bytes = execution_time.to_bytes(2, 'big')  # 执行时间（大端字节序）

        # 构建数据部分，包括舵机 ID、指令类型、地址、目标位置和执行时间
        data = [servo_id, length, instruction_type, address] + list(position_bytes) + list(execution_time_bytes)

        # 计算校验和：对所有数据字节求和并取反
        checksum = ~(sum(data) & 0xFF) & 0xFF  # 将所有字节相加后取反，并与0xFF按位与

        # 构造完整的指令包
        command = header + bytes(data) + bytes([checksum])

        # 发送指令
        if self.ser:
            self.ser.write(command)
            self.ser.flush()
            self.get_logger().info(f'Sent angle {angle}° (converted to position {position}) with execution time {execution_time} to servo {servo_id}.')
        else:
            self.get_logger().error("Serial port is not initialized.")




def main(args=None):
    rclpy.init(args=args)
    gimbal_controller = GimbalController()

    try:
        rclpy.spin(gimbal_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保串口总是被关闭
        if gimbal_controller.ser:
            gimbal_controller.ser.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
