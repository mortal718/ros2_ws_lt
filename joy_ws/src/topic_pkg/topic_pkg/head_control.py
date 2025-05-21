import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import time 

class HeadControl(Node):
    def __init__(self):
        super().__init__('head_control')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.publisher = self.create_publisher(String, '/eye_expression', 10)
        self.gimbal_publisher = self.create_publisher(String, '/gimbal_control_topic', 10)
        
        # 添加变量来跟踪上一次的轴值和按钮状态
        self.previous_axis_value = 0
        self.activate_button_id = 6  # 激活按钮的ID
        self.blink_button_id = 8     # 眨眼按钮的ID
        self.initial_L1_button_id = 4  # 初始化按钮的ID
        self.gimbal_up_down_axis = 7  # 云台上下轴的ID
        self.previous_blink_state = False  # 跟踪眨眼按钮的上一次状态
        self.previous_L1_state = False    # 跟踪L1按钮的上一次状态
        self.previous_gimbal_state = False  # 跟踪云台按钮的上一次状态
        self.current_direction = 'center'  # 跟踪当前眼睛方向
        self.current_vertical_angle = 0  # 跟踪云台当前垂直角度
        self.current_horizontal_angle = 0  # 跟踪云台当前水平角度
        
        # 添加云台运动时间参数
        self.horizontal_move_time = 1  # 水平转动时间(ms)
        self.vertical_move_time = 200    # 垂直转动时间(ms)
        self.reset_move_time = 150     # 重置动作时间(ms)
        
        # 添加防误触相关变量
        self.last_expression_time = self.get_clock().now()  # 云台结束时间
        self.last_expression_time2 = self.get_clock().now()  # 表情结束时间
        self.expression_cooldown = 0.5 # 冷却时间（秒）

    def publish_gimbal(self, servo_id, angle, move_time):
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_expression_time).nanoseconds / 1e9  # 转换为秒
        
        # 检查是否在冷却时间内
        if time_diff < self.expression_cooldown:
            self.get_logger().info('云台转动冷却中...')
            return
            
        gimbal_msg = String()
        gimbal_msg.data = f'{servo_id} {angle} {move_time}'
        self.gimbal_publisher.publish(gimbal_msg)
        self.get_logger().info(f'Published gimbal command: {gimbal_msg.data}')
        self.last_expression_time = self.get_clock().now()

    def publish_expression(self, expression):
        current_time2 = self.get_clock().now()
        time_diff2 = (current_time2 - self.last_expression_time2).nanoseconds / 1e9  # 转换为秒
        
        # 检查是否在冷却时间内
        if time_diff2 < self.expression_cooldown:
            self.get_logger().info('表情冷却中...')
            return   
        msg = String()
        msg.data = expression
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {expression}')
        self.last_expression_time2 = self.get_clock().now()

    def joy_callback(self, msg):
        # 检查激活按钮是否被按下
        if not msg.buttons[self.activate_button_id]:
            return  # 如果激活按钮没有被按下，直接返回
        
        # 处理云台上下按钮
        current_gimbal_value = msg.axes[self.gimbal_up_down_axis]
        if current_gimbal_value != 0 and not self.previous_gimbal_state:
            if current_gimbal_value == 1:               
                self.publish_gimbal(2, 25, self.vertical_move_time)
                self.current_vertical_angle = 25
            elif current_gimbal_value == -1:               
                self.publish_gimbal(2, 10, self.vertical_move_time)
                self.current_vertical_angle = 10
        self.previous_gimbal_state = (current_gimbal_value != 0)
            
        # 处理L1按钮
        current_L1_state = msg.buttons[self.initial_L1_button_id]
        if current_L1_state and not self.previous_L1_state:
            self.publish_expression('center')
            self.current_direction = 'center'
            
            self.publish_gimbal(1, 0, self.reset_move_time)
            self.current_horizontal_angle = 0

        self.previous_L1_state = current_L1_state

        # 处理眨眼按钮
        current_blink_state = msg.buttons[self.blink_button_id]
        if current_blink_state and not self.previous_blink_state:
            self.publish_expression('blink')
        self.previous_blink_state = current_blink_state

        # 处理水平方向控制
        axis_value = msg.axes[6]
        if axis_value != self.previous_axis_value and axis_value != 0:
            if axis_value == 1:  # 向右
                self.publish_expression('right')
                self.current_direction = 'right'
                self.publish_gimbal(1, 20, self.horizontal_move_time)
                self.current_horizontal_angle = 20
            elif axis_value == -1:  # 向左               
                self.publish_expression('left')
                self.current_direction = 'left'
                
                self.publish_gimbal(1, -20, self.horizontal_move_time)
                self.current_horizontal_angle = -20
        self.previous_axis_value = axis_value

def main(args=None):
    rclpy.init(args=args)
    head_control = HeadControl()
    rclpy.spin(head_control)
    head_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()