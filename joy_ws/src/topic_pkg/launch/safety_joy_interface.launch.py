import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments for navigation velocity output and velocity command output
    navigation_vel_out = LaunchConfiguration('navigation_vel_out', default='/cmd_vel_nav')
    vel_cmd_out = LaunchConfiguration('vel_cmd_out', default='/cmd_vel_des')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('navigation_vel_out', default_value='/cmd_vel_nav', description='Navigation velocity output topic'),
        DeclareLaunchArgument('vel_cmd_out', default_value='/cmd_vel_des', description='Velocity command output topic'),

        # Log information about the launch
        LogInfo(condition=None, msg="Starting safety_joy_interface node"),

        # Launch the safety_joy_interface node
        Node(
            package='topic_pkg',
            executable='safety_joy_interface',
            name='safety_joy_interface',
            output='screen',
            respawn=True,
            parameters=[{'navigation_frequency': 20.0}],
            remappings=[
                #self.des_vel_cmd_sub_ = self.create_subscription(Twist, 'vel_cmd_des', self.cb_des_vel_cmd, 10)#AUTONOMOUS
                ('vel_cmd_des', navigation_vel_out), #default='/cmd_vel_nav'
                ('joystick_mode', '/joystick_teleop/control_status'),
                ('joystick_ctrl_cmd', '/joystick_teleop/cmd_vel'),
                ('cmd_vel', vel_cmd_out) #default='/cmd_vel_des'
            ]
        ),
    ])
