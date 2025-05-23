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


    return LaunchDescription([




        Node(
            package='topic_pkg',  
            executable='avoidance_controller',  
            name='avoidance_controller',
            output='screen',
            parameters=[
                {'max_linear_vel': 0.3},  
                {'max_side_vel': 0.4},  
                {'max_angular_vel': 0.9},  
                {'move_duration': 2.0},  
                {'activate_button': 6},
                {'r1_button': 5}
            ],
            remappings=[
                ('cmd_vel_out', '/cmd_vel_mux/avoidance_controller')
            ],
        ),
  
    ])
