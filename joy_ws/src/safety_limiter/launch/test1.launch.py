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
    cmd_vel_in = LaunchConfiguration('cmd_vel_in', default='/cmd_vel_des')
    cmd_vel_out = LaunchConfiguration('cmd_vel_out', default='/cmd_vel_mux/cmd_vel_safety')
    navigation_vel_out = LaunchConfiguration('navigation_vel_out', default='/cmd_vel_nav')
    vel_cmd_out = LaunchConfiguration('vel_cmd_out', default='/cmd_vel_des')
    dev = LaunchConfiguration('dev', default='/dev/input/js0')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.4')
    max_side_vel = LaunchConfiguration('max_side_vel', default='0.4')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='0.6')
    use_side_vel = LaunchConfiguration('use_side_vel', default='False')
    limit_out = LaunchConfiguration('limit_out', default='/cmd_vel_mux/joystick_controller')



    default_config_locks = os.path.join(get_package_share_directory('twist_mux'),
                                        'config', 'twist_mux_locks.yaml')
    default_config_topics = os.path.join(get_package_share_directory('safety_limiter'),
                                  'config', 'cmd_mux_config.yaml')
    default_config_joystick = os.path.join(get_package_share_directory('twist_mux'),
                                           'config', 'joystick.yaml')



    return LaunchDescription([


        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='/cmd_vel',
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'),



        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[

                LaunchConfiguration('config_topics')]
        ),



        # SAFETY LIMITER：包含 safety_exp_atc_hatshop.launch
        
        # IncludeLaunchDescription(
        #     FindPackageShare('safety_limiter').find('launch/safety_exp_atc_hatshop.launch'),
        #     launch_arguments={'cmd_vel_in': cmd_vel_in, 'cmd_vel_out': cmd_vel_out}.items()
        # ),


        # SAFETY JOY INTERFACE：包含 safety_joy_interface.launch.py
        IncludeLaunchDescription(
            os.path.join(get_package_share_directory('safety_limiter'),'launch/safety_joy_interface.launch.py'),
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


    
        
    ])
