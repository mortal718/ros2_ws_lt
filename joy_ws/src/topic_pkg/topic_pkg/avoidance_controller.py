#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

class AvoidanceController(Node):
    def __init__(self):
        super().__init__('avoidance_controller')
        
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel_out',
            10)
        
        # 声明参数
        self.declare_parameter('max_linear_vel', 0.3)  #
        self.declare_parameter('max_side_vel', 0.4)  #
        self.declare_parameter('max_angular_vel',0.9)  #  max:0.9
        self.declare_parameter('move_duration', 2.0)  # 侧移持续时间(秒)
        
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_side_vel = self.get_parameter('max_side_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.move_duration = self.get_parameter('move_duration').value
        
        # 按钮ID
        self.declare_parameter('activate_button', 6)
        self.declare_parameter('r1_button', 5)
        
        self.activate_button = self.get_parameter('activate_button').value
        self.r1_button = self.get_parameter('r1_button').value
        
        # 状态变量
        self.is_avoiding = False
        self.previous_r1_state = False
        self.avoidance_start_time = None
        
        self.get_logger().info('Avoidance controller started, current state: Avoidance mode OFF')
    
    def joy_callback(self, msg):
        # 检查按钮状态
        if len(msg.buttons) <= max(self.activate_button, self.r1_button):
            self.get_logger().warn('Button index out of range')
            return
        
        activate_pressed = msg.buttons[self.activate_button] == 1
        current_r1_state = msg.buttons[self.r1_button] == 1
        
        # 创建Twist消息
        cmd_vel = Twist()
        
        # 检测激活键+R1的组合按键触发
        if activate_pressed and current_r1_state and not self.previous_r1_state:
            self.is_avoiding = not self.is_avoiding
            if self.is_avoiding:
                self.avoidance_start_time = self.get_clock().now()
                self.get_logger().info('Entering avoidance mode - Side move + rotation')
            else:
                self.get_logger().info('Exiting avoidance mode')
        
        # 在避让模式中执行复合动作
        if self.is_avoiding:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.avoidance_start_time).nanoseconds / 1e9
            
            if elapsed_time < self.move_duration:
                # 同时进行侧向移动和旋转
                # 侧向移动速度(45度方向)
                cmd_vel.linear.x = self.max_linear_vel 
                cmd_vel.linear.y = -self.max_side_vel
                cmd_vel.angular.z = self.max_angular_vel            
                self.cmd_vel_pub.publish(cmd_vel)
            else:
                self.get_logger().info('avoidance mode finished')
        self.previous_r1_state = current_r1_state

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()