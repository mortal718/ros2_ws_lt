
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from interface_pkg.msg import CtrlStatus
import time

class JoyTeleopNode(Node):

    def __init__(self):
        super().__init__('joystick_teleop_omni')

        # Declare and get parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('activate_button_id', 6),
                ('heartbeat_button_id', 7),
                ('axis_linear_id', 1),
                ('axis_side_id', 2),
                ('axis_angular_id', 3),
                ('max_linear_vel', 0.5),
                ('max_side_vel', 0.5),
                ('max_angular_vel', 0.8),
                ('use_side_vel', False),
                ('timeout_duration', 2.0),
                ('button_circle_id', 1),
                ('button_cross_id', 0),
                ('button_rectangle_id', 3),
                ('button_triangle_id', 2),
                ('button_L1_id', 4),  
                ('button_R1_id', 5),  
                ('cmd_circle', 'Limited Mode On'),
                ('cmd_cross', 'EMERGENCY MODE ALREADY ON!! Possible to recover with L1+R1+Velocity Mode...'),
                ('cmd_rectangle', 'Velocity Control Mode On'),
                ('cmd_triangle', 'Mission starts, Autonomous Mode On'),
                ('lock_period', 1.0),
            ]
        )

        # Assign parameters to variables
        self.activate_button_id = self.get_parameter('activate_button_id').value
        self.heartbeat_button_id = self.get_parameter('heartbeat_button_id').value
        self.axis_linear_id = self.get_parameter('axis_linear_id').value
        self.axis_side_id = self.get_parameter('axis_side_id').value
        self.axis_angular_id = self.get_parameter('axis_angular_id').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_side_vel = self.get_parameter('max_side_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.use_side_vel = self.get_parameter('use_side_vel').value
        self.timeout_duration = self.get_parameter('timeout_duration').value
        self.lock_period = self.get_parameter('lock_period').value

        # Mode buttons and commands
        self.button_circle_id = self.get_parameter('button_circle_id').value
        self.button_cross_id = self.get_parameter('button_cross_id').value
        self.button_rectangle_id = self.get_parameter('button_rectangle_id').value
        self.button_triangle_id = self.get_parameter('button_triangle_id').value
        self.button_L1_id = self.get_parameter('button_L1_id').value
        self.button_R1_id = self.get_parameter('button_R1_id').value


        self.cmd_circle = self.get_parameter('cmd_circle').value
        self.cmd_cross = self.get_parameter('cmd_cross').value
        self.cmd_rectangle = self.get_parameter('cmd_rectangle').value
        self.cmd_triangle = self.get_parameter('cmd_triangle').value

        # Publishers and subscribers
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.limit_pub = self.create_publisher(Twist, 'limit_vel', 10)
        self.status_pub = self.create_publisher(CtrlStatus, 'control_status', 10)
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Variables
        self.last_time = self.get_clock().now()
        self.activate_button_pressed = False
        self.current_mode = 'VELOCITY'
        self.twist = Twist()

        # Button state variables
        self.button_circle_on = False
        self.button_cross_on = False
        self.button_rectangle_on = True
        self.button_triangle_on = False

        self.get_logger().info('Joystick Teleop Node Initialized')

    def joy_callback(self, msg: Joy):
        """Handle joystick input"""
        # Check activation button
        if msg.buttons[self.activate_button_id] == 1:
            self.activate_button_pressed = True
        else:
            if self.activate_button_pressed:
                self.twist = Twist()
                self.vel_pub.publish(self.twist)
                self.activate_button_pressed = False

        # Handle motion based on mode
        if self.activate_button_pressed and not self.button_cross_on and not self.button_circle_on:
            self.twist.linear.x = self.max_linear_vel * msg.axes[self.axis_linear_id]
            self.twist.linear.y = self.max_side_vel * msg.axes[self.axis_side_id] if self.use_side_vel else 0.0
            self.twist.angular.z = self.max_angular_vel * msg.axes[self.axis_angular_id]


        if self.activate_button_pressed and not self.button_cross_on and self.button_circle_on:
            #self.get_logger().info('aaa')
            self.twist.linear.x = 0.5 * msg.axes[self.axis_linear_id]
            self.twist.linear.y = 0.5 * msg.axes[self.axis_side_id] if self.use_side_vel else 0.0
            self.twist.angular.z = 1.2 * msg.axes[self.axis_angular_id]

        # Handle mode switching
        self.switch_mode(msg)

        # Publish based on mode
        if self.activate_button_pressed:
            if self.button_rectangle_on:
                self.vel_pub.publish(self.twist)
            elif self.button_circle_on:
                self.limit_pub.publish(self.twist)

    def switch_mode(self, msg: Joy):
        """Switch control modes based on button presses"""
        if msg.buttons[self.button_cross_id] == 1:
            if not self.button_cross_on:
                self.get_logger().error(self.cmd_cross)
                self.current_mode = 'EMERGENCY'
                self.button_cross_on = True
                self.button_circle_on = False
                self.button_rectangle_on = False
                self.button_triangle_on = False
        elif msg.buttons[self.button_circle_id] == 1:
            if not self.button_circle_on and not self.button_cross_on:
                self.get_logger().warn(self.cmd_circle)
                self.current_mode = 'LIMITED'
                self.button_circle_on = True
                self.button_rectangle_on = False
                self.button_triangle_on = False
        elif msg.buttons[self.button_rectangle_id] == 1 and msg.buttons[self.button_L1_id] == 0 and msg.buttons[self.button_R1_id] == 0:
            if not self.button_rectangle_on and not self.button_cross_on:
                self.get_logger().info(self.cmd_rectangle)
                self.current_mode = 'VELOCITY'
                self.button_rectangle_on = True
                self.button_circle_on = False
                self.button_triangle_on = False
        elif msg.buttons[self.button_triangle_id] == 1:
            if not self.button_triangle_on and not self.button_cross_on:
                self.get_logger().info(self.cmd_triangle)
                self.current_mode = 'AUTONOMOUS'
                self.button_triangle_on = True
                self.button_circle_on = False
                self.button_rectangle_on = False

        elif msg.buttons[self.button_L1_id] == 1 and msg.buttons[self.button_R1_id] == 1 and msg.buttons[self.button_rectangle_id] == 1:
            #self.get_logger().info("Combination keys pressed")
            if  self.button_cross_on:
                self.get_logger().warn("Recover from EMERGENCY MODE! Velocity Mode On!")
                self.current_mode = 'VELOCITY'
                self.button_cross_on = False
                self.button_rectangle_on = True
                self.button_circle_on = False
                self.button_triangle_on = False

    def run(self):
        """Main loop to handle timeouts and publish status"""
        timer_period = 0.05  # seconds
        self.create_timer(timer_period, self.publish_status)

    def publish_status(self):
        """Publish the current control mode as a status message"""
        status = CtrlStatus()
        status.header.stamp = self.get_clock().now().to_msg()

        if self.button_cross_on:
            status.status = CtrlStatus.EMERGENCY
        elif self.button_triangle_on:
            status.status = CtrlStatus.AUTONOMOUS
        elif self.button_rectangle_on:
            status.status = CtrlStatus.VELOCITY
        elif self.button_circle_on:
            status.status = CtrlStatus.LIMITED

        self.status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


