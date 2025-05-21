from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments


    return LaunchDescription([

        DeclareLaunchArgument('dev', default_value='/dev/input/js0', description='Joystick device path'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/joystick_teleop/cmd_vel', description='Command velocity topic'),
        DeclareLaunchArgument('limit_out', default_value='/joystick_teleop/limit_vel', description='Limit velocity topic'),
        #DeclareLaunchArgument('cmd_vel_out',  description='Command velocity topic'),
        #DeclareLaunchArgument('limit_out', description='Limit velocity topic'),
        DeclareLaunchArgument('max_linear_vel', default_value='1.0', description='Maximum linear velocity'),
        DeclareLaunchArgument('max_side_vel', default_value='1.0', description='Maximum side velocity'),
        DeclareLaunchArgument('max_angular_vel', default_value='1.0', description='Maximum angular velocity'),
        DeclareLaunchArgument('timeout_duration', default_value='1.0', description='Timeout duration for joystick control'),
        DeclareLaunchArgument('use_side_vel', default_value='True', description='Enable side velocity control'),

        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {'dev': LaunchConfiguration('dev')}
            ],
            # remappings=[
            #     ('/joy', 'joystick_teleop/joy')
            # ],
            respawn=True,
        ),

        # Joystick teleop node
        Node(
            package='topic_pkg',  # 替换为实际的包名
            executable='joystick_teleop_omni',
            name='joystick_teleop',
            #namespace='joystick_teleop',
            parameters=[
                {'max_linear_vel': LaunchConfiguration('max_linear_vel')},
                {'max_side_vel': LaunchConfiguration('max_side_vel')},
                {'max_angular_vel': LaunchConfiguration('max_angular_vel')},
                {'timeout_duration': LaunchConfiguration('timeout_duration')},
                {'use_side_vel': LaunchConfiguration('use_side_vel')},
                {'activate_button_id': 6},
                {'heartbeat_button_id': 7},
                {'axis_linear_id': 1},
                {'axis_side_id': 0},
                {'axis_angular_id': 3},
                {'lock_period': 0.1},
                {'button_circle_id': 1},
                {'cmd_circle': 'Limited Motion Mode On'},
                {'button_cross_id': 0},
                {'cmd_cross': 'Emergency Mode On, Motion Off'},
                {'button_rectangle_id':  3},
                {'cmd_rectangle': 'Velocity Control Mode On'},
                {'button_triangle_id': 2},
                {'cmd_triangle': 'Mission starts, Autonomous Mode On'},
                {'button_L1_id': 4},
                {'button_R1_id': 5},
            ],
            remappings=[
                #('/joystick_teleop/cmd_vel', LaunchConfiguration('cmd_vel_out')),
                #('/joystick_teleop/limit_vel', LaunchConfiguration('limit_out')),
                ('cmd_vel', LaunchConfiguration('cmd_vel_out')),
                ('limit_vel', LaunchConfiguration('limit_out')),
                ('control_status', 'joystick_teleop/control_status'),
            ],
            output='screen',
        ),


    ])
