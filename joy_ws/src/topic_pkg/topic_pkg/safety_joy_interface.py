import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from interface_pkg.msg import CtrlStatus
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs
from std_msgs.msg import String

class SafetyJoyInterface(Node):

    def __init__(self):
        super().__init__('safety_joy_interface')


        self.joystate_ = CtrlStatus()  # joystate_初始化
        self.joy_vel_ = Twist()  # 空的初始值
        self.des_vel_cmd_ = Twist()

        # 获取参数（ROS 2 获取参数的方式）
        self.declare_parameter("navigation_frequency", 20.0)
        self.declare_parameter("planner_global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        self.hz = self.get_parameter("navigation_frequency").get_parameter_value().double_value
        self.planner_global_frame = self.get_parameter("planner_global_frame").get_parameter_value().string_value
        self.robot_frame = self.get_parameter("robot_frame").get_parameter_value().string_value

        self.planner_velocity = Twist()
        self.zero_velocity = Twist()

        # 创建发布者和订阅者
        self.vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.des_vel_cmd_sub_ = self.create_subscription(Twist, 'vel_cmd_des', self.cb_des_vel_cmd, 10)#AUTONOMOUS
        self.joystate_sub_ = self.create_subscription(CtrlStatus, 'joystick_mode', self.cb_joy_state, 10)
        self.joy_vel_sub_ = self.create_subscription(Twist, 'joystick_ctrl_cmd', self.cb_joy_ctrl, 10)#VELOCITY

        # TF 监听器，增加缓存时间
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))  # 增加缓存时间
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_status = None  # 用于记录上次的状态

        self.get_logger().info('SafetyJoyInterface node started.')

    def cb_joy_state(self, joy_msg):
        #self.get_logger().info('Joystick connected!')
        self.joystate_ = joy_msg

    def cb_joy_ctrl(self, joy_cmd):
        self.joy_vel_ = joy_cmd

    def cb_des_vel_cmd(self, des_vel_msg):
        self.get_logger().info('First velocity command from navigation module received!')
        self.des_vel_cmd_ = des_vel_msg

    def publish_velocity(self):
        # 发布速度
        self.vel_pub_.publish(self.planner_velocity)

    def run(self):
        # 使用定时器持续发布速度并处理回调
        timer = self.create_timer(1.0 / self.hz, self.timer_callback)

    def timer_callback(self):
        if self.joystate_.status == CtrlStatus.AUTONOMOUS:
            self.planner_velocity.linear.x = self.des_vel_cmd_.linear.x
            self.planner_velocity.linear.y = self.des_vel_cmd_.linear.y
            self.planner_velocity.angular.z = self.des_vel_cmd_.angular.z
            if self.last_status != CtrlStatus.AUTONOMOUS:
                self.get_logger().info(f"Copying desired velocity commands from navigation module in {self.robot_frame} frame")
            self.last_status = CtrlStatus.AUTONOMOUS

        elif self.joystate_.status == CtrlStatus.VELOCITY:
            self.planner_velocity.linear.x = self.joy_vel_.linear.x
            self.planner_velocity.linear.y = self.joy_vel_.linear.y
            self.planner_velocity.angular.z = self.joy_vel_.angular.z
            if self.last_status != CtrlStatus.VELOCITY:
                self.get_logger().info("Controlling robot velocity via Joystick Velocity Mode!")
            self.last_status = CtrlStatus.VELOCITY

        elif self.joystate_.status == CtrlStatus.EMERGENCY:
            # self.planner_velocity = self.zero_velocity
            self.planner_velocity.linear.x = 0.0
            self.planner_velocity.linear.y = 0.0
            self.planner_velocity.angular.z = 0.0
            if self.last_status != CtrlStatus.EMERGENCY:
                self.get_logger().warn("Robot velocity FROZEN due to Joystick Emergency Mode ON!")
            self.last_status = CtrlStatus.EMERGENCY
        
        else:
            self.planner_velocity.linear.x = 0.0
            self.planner_velocity.linear.y = 0.0
            self.planner_velocity.angular.z = 0.0

        self.publish_velocity()


def main(args=None):
    rclpy.init(args=args)

    safety_joy_interface = SafetyJoyInterface()

    try:
        safety_joy_interface.run()
        rclpy.spin(safety_joy_interface)
    except KeyboardInterrupt:
        pass
    finally:
        safety_joy_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
