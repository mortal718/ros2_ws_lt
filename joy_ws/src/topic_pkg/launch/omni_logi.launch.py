from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 动态参数声明
    return LaunchDescription([
        # Declare arguments for dynamic input
        DeclareLaunchArgument('dev', default_value='/dev/input/js0',description='Joystick device input path'),
        DeclareLaunchArgument('cmd_vel_out',default_value='/cmd_vel',description='Output topic for velocity commands'),
        DeclareLaunchArgument('limit_out',default_value='/limit_vel',description='Output topic for limited velocity commands'),
        DeclareLaunchArgument('max_linear_vel',default_value='0.5',description='Maximum linear velocity'),
        DeclareLaunchArgument('max_side_vel',default_value='0.0',description='Maximum side velocity'),
        DeclareLaunchArgument('max_angular_vel',default_value='1.0',description='Maximum angular velocity'),
        DeclareLaunchArgument('timeout_duration',default_value='1.5',description='Timeout duration for safety'),
        DeclareLaunchArgument('use_side_vel',default_value='False',description='Enable side velocity control'),

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {'dev': LaunchConfiguration('dev')}
            ],
            remappings=[
                ('/joy', '/joystick_teleop/joy')
            ],
        ),

        # Joystick teleop node
        Node(
            package='topic_pkg',  # 替换为实际的包名
            executable='joystick_teleop_omni',
            name='node',
            parameters=[
                {'max_linear_vel': LaunchConfiguration('max_linear_vel')},
                {'max_side_vel': LaunchConfiguration('max_side_vel')},
                {'max_angular_vel': LaunchConfiguration('max_angular_vel')},
                {'timeout_duration': LaunchConfiguration('timeout_duration')},
                {'use_side_vel': LaunchConfiguration('use_side_vel')},
                {'activate_button_id': 6},
                {'heartbeat_button_id': 7},
                {'axis_linear_id': 1},
                {'axis_side_id': 2},
                {'axis_angular_id': 0},
                {'lock_period': 0.1},
                {'button_circle_id': 2},
                {'cmd_circle': 'Limited Motion Mode On'},
                {'button_cross_id': 1},
                {'cmd_cross': 'Emergency Mode On, Motion Off'},
                {'button_rectangle_id': 0},
                {'cmd_rectangle': 'Velocity Control Mode On'},
                {'button_triangle_id': 3},
                {'cmd_triangle': 'Mission starts, Autonomous Mode On'},
                {'button_L1_id': 4},
                {'button_R1_id': 5},
            ],
            remappings=[
                ('/joystick_teleop/cmd_vel', LaunchConfiguration('cmd_vel_out')),
                ('/joystick_teleop/limit_vel', LaunchConfiguration('limit_out')),
            ],
            output='screen',
        ),
    ])
