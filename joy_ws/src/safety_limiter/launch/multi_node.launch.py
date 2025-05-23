import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # 声明启动文件的参数
    cmd_vel_in = LaunchConfiguration('cmd_vel_in', default='/cmd_vel_des_new')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out', default='/cmd_vel_mux/cmd_vel_safety')
    navigation_vel_out = LaunchConfiguration('navigation_vel_out', default='/cmd_vel_navigation')
    vel_cmd_out = LaunchConfiguration('vel_cmd_out', default='/cmd_vel_des')
    dev = LaunchConfiguration('dev', default='/dev/input/js0')
    
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.5')
    max_side_vel = LaunchConfiguration('max_side_vel', default='0.5')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='0.8')

    # max_linear_vel = LaunchConfiguration('max_linear_vel', default='1.0')
    # max_side_vel = LaunchConfiguration('max_side_vel', default='1.0')
    # max_angular_vel = LaunchConfiguration('max_angular_vel', default='1.0')

    use_side_vel = LaunchConfiguration('use_side_vel', default='True')
    limit_out = LaunchConfiguration('limit_out', default='/cmd_vel_mux/joystick_controller')



    default_config_locks = os.path.join(get_package_share_directory('twist_mux'),
                                        'config', 'twist_mux_locks.yaml')
    default_config_topics1 = os.path.join(get_package_share_directory('safety_limiter'),
                                  'config', 'cmd_mux_config.yaml')
    default_config_topics2 = os.path.join(get_package_share_directory('safety_limiter'),
                                  'config', 'des_avoidance.yaml')
    default_config_joystick = os.path.join(get_package_share_directory('twist_mux'),
                                           'config', 'joystick.yaml')


    return LaunchDescription([


        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics1',
            default_value=default_config_topics1,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_topics2',
            default_value=default_config_topics2,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out1',
            default_value='/cmd_vel',
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'cmd_vel_out2',
            default_value='/cmd_vel_des_new',
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation time'),



        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux_1',
            output='screen',
            remappings={('cmd_vel_out', LaunchConfiguration('cmd_vel_out1'))},
            parameters=[
                LaunchConfiguration('config_topics1')
                ]
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux_2', 
            output='screen',
            remappings={('cmd_vel_out', LaunchConfiguration('cmd_vel_out2'))},
            parameters=[
                LaunchConfiguration('config_topics2')
                ]
        ),



        # SAFETY LIMITER：包含 safety_exp_atc_hatshop.launch.py
        
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('safety_limiter'),'launch/safety_exp_atc_hatshop.launch.py'),
            launch_arguments={'cmd_vel_in': cmd_vel_in, 'cmd_vel_out': cmd_vel_out}.items()
        ),

        # SAFETY JOY INTERFACE：包含 safety_joy_interface.launch.py
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('topic_pkg'),'launch/safety_joy_interface.launch.py'),
            launch_arguments={'navigation_vel_out': navigation_vel_out, 'vel_cmd_out': vel_cmd_out}.items()
        ),

        # JOYSTICK CONTROLLER：包含 joystick_teleop 的 nav_modal.launch.py
        IncludeLaunchDescription(
            # FindPackageShare('joystick_teleop').find('launch/nav_modal.launch'),
            os.path.join(get_package_share_directory('topic_pkg'),'launch/omni_ps5.launch.py'),

            launch_arguments={
                'dev': dev,
                'max_linear_vel': max_linear_vel,
                'max_side_vel': max_side_vel,
                'max_angular_vel': max_angular_vel,
                'use_side_vel': use_side_vel,
                'cmd_vel_out': '/joystick_teleop/cmd_vel',  # 假设不使用导航模块
                'limit_out': limit_out,
            }.items()
        ),


        Node(
            package='topic_pkg',
            executable='head_control',
            output='screen'
        ),

        Node(
            package='topic_pkg',  
            executable='avoidance_controller',  
            name='avoidance_controller',
            output='screen',
            parameters=[
                {'max_linear_vel': 0.0},  
                {'max_side_vel': 0.4},  
                {'max_angular_vel': 0.6},  
                {'move_duration': 2.0},  
                {'activate_button': 6},
                {'r1_button': 5}
            ],
            remappings=[
                ('cmd_vel_out', '/cmd_vel_mux/avoidance_controller')
            ],
        ),
    
        
    ])
