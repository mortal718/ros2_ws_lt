#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

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
        self.declare_parameter('avoidance_speed', 0.5)
        self.avoidance_speed = self.get_parameter('avoidance_speed').value
        
        # 按钮ID
        self.declare_parameter('activate_button', 6)
        self.declare_parameter('r1_button', 5)
        
        self.activate_button = self.get_parameter('activate_button').value
        self.r1_button = self.get_parameter('r1_button').value
        
        # 状态变量
        self.is_avoiding = False
        self.previous_r1_state = False
        
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
                self.get_logger().info('Entering avoidance mode')
            else:
                self.get_logger().info('Exiting avoidance mode')
        
        # 只在避让模式中发送旋转指令
        if self.is_avoiding:
            cmd_vel.angular.z = self.avoidance_speed
            self.cmd_vel_pub.publish(cmd_vel)
        
        self.previous_r1_state = current_r1_state

def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()