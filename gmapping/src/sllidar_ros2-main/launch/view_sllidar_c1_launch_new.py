#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port_1 = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_port_2 = LaunchConfiguration('serial_port', default='/dev/ttyUSB1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id_1 = LaunchConfiguration('frame_id', default='laser_1')
    frame_id_2 = LaunchConfiguration('frame_id', default='laser_2')
#    scan_frequency_a = LaunchConfiguration('scan_frequency', default=9.0)
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    cut_angle = LaunchConfiguration('cut_angle', default='true')
    right_degrees_1 = LaunchConfiguration('right_degrees', default='0')
    left_degrees_1 = LaunchConfiguration('left_degrees', default='180')
    right_degrees_2 = LaunchConfiguration('right_degrees', default='180')
    left_degrees_2 = LaunchConfiguration('left_degrees', default='360')
    rviz_config_dir = os.path.join(
            get_package_share_directory('sllidar_ros2'),
            'rviz',
            'sllidar_ros2.rviz')


    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

#        DeclareLaunchArgument(
#            'serial_port',
#            default_value=serial_port_1,
#           description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
#        DeclareLaunchArgument(
#            'frame_id',
#            default_value=frame_id_1,
#            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            remappings = [
            ('scan', 'scan_1')],
            name='sllidar_node1',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port_1, 
                         'serial_baudrate': serial_baudrate, 
#                         'tcp_ip':'192.168.0.7',
#                         'udp_ip':'192.168.11.2',
                         'frame_id': frame_id_1,
                         'timestamp_type': 'realtime',
                         'inverted': inverted, 
#                         'cut_angle': cut_angle,
#	                 'right_degrees': right_degrees_1,
#	                 'left_degrees': left_degrees_1,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode
                         }],
            output='screen'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            remappings = [
            ('scan', 'scan_2')],
            name='sllidar_node2',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port_2, 
                         'serial_baudrate': serial_baudrate, 
 #                        'tcp_ip':'192.168.0.8',
 #                        'udp_ip':'192.168.11.3',
                         'frame_id': frame_id_2,
                         'timestamp_type': 'realtime',
                         'inverted': inverted, 
#                         'cut_angle': cut_angle,
#	                 'right_degrees': right_degrees_2,
#	                 'left_degrees': left_degrees_2,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode
                         #'scan_frequency': scan_frequency_a
                         }],
            output='screen'),
            
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])

