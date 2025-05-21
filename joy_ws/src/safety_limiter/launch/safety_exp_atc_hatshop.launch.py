from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 声明参数
    declared_arguments = [
        DeclareLaunchArgument('max_linear_velocity', default_value='0.5'),
        DeclareLaunchArgument('max_angular_velocity', default_value='0.9'),
        DeclareLaunchArgument('max_linear_acceleration', default_value='1.0'),
        DeclareLaunchArgument('max_angular_acceleration', default_value='1.0'),
        DeclareLaunchArgument('disable_limiter', default_value='false'),
        DeclareLaunchArgument('cmd_vel_in', default_value='/cmd_vel_des'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_mux/cmd_vel_safety'),
        #DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel'),
        DeclareLaunchArgument('disable_front_laser', default_value='false'),
        DeclareLaunchArgument('disable_rear_laser', default_value='false'),
        DeclareLaunchArgument('disable_left_laser', default_value='false'),
        DeclareLaunchArgument('disable_right_laser', default_value='false'),
    ]

    # # URG激光雷达的位置参数
    # urg_pose_01 = ['0.21', '0.0', '0.11', '0.01', '0', '0']
    # urg_pose_02 = ['-0.21', '0.0', '0.11', '-3.14', '0', '0']
    # urg_pose_03 = ['0.0', '0.21', '0.11', '1.61', '0', '0']
    # urg_pose_04 = ['0.0', '-0.21', '0.11', '-1.56', '0', '0']

    # # TF发布节点
    # tf_nodes = [
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='scan_01_tf',
    #         arguments=urg_pose_01 + ['base_link', 'laser1']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='scan_02_tf',
    #         arguments=urg_pose_02 + ['base_link', 'laser2']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='scan_03_tf',
    #         arguments=urg_pose_03 + ['base_link', 'laser3']
    #     ),
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='scan_04_tf',
    #         arguments=urg_pose_04 + ['base_link', 'laser4']
    #     ),
    # ]

    # # URG激光雷达节点
    # urg_nodes = [
    #     Node(
    #         package='urg_node',
    #         executable='urg_node_driver',
    #         name='urg_front',
    #         parameters=[{
    #             'frame_id': 'laser1',
    #             'angle_min': -1.5,
    #             'angle_max': 1.5,
    #             'intensity': False
    #         }],
    #         condition=UnlessCondition(LaunchConfiguration('disable_front_laser'))
    #     ),
    #     Node(
    #         package='urg_node',
    #         executable='urg_node_driver',
    #         name='urg_rear',
    #         parameters=[{
    #             'frame_id': 'laser2',
    #             'angle_min': -1.5,
    #             'angle_max': 1.5,
    #             'intensity': False
    #         }],
    #         condition=UnlessCondition(LaunchConfiguration('disable_rear_laser'))
    #     ),
    #     Node(
    #         package='urg_node',
    #         executable='urg_node_driver',
    #         name='urg_left',
    #         parameters=[{
    #             'frame_id': 'laser3',
    #             'angle_min': -1.5,
    #             'angle_max': 1.5,
    #             'intensity': False
    #         }],
    #         condition=UnlessCondition(LaunchConfiguration('disable_left_laser'))
    #     ),
    #     Node(
    #         package='urg_node',
    #         executable='urg_node_driver',
    #         name='urg_right',
    #         parameters=[{
    #             'frame_id': 'laser4',
    #             'angle_min': -1.5,
    #             'angle_max': 1.5,
    #             'intensity': False
    #         }],
    #         condition=UnlessCondition(LaunchConfiguration('disable_right_laser'))
    #     ),
    # ]

    # # 激光扫描组装节点
    # laser_assembler_nodes = [
    #     Node(
    #         package='laser_assembler',
    #         executable='laser_scan_assembler',
    #         name='scan2cloud',
    #         parameters=[{
    #             'max_scans': 100,
    #             'fixed_frame': 'base_link',
    #             'tf_cache_time_secs': 5.0
    #         }],
    #         remappings=[('scan', '/scan')]
    #     ),
    #     Node(
    #         package='laser_preprocess',
    #         executable='periodic_assemble_scans',
    #         name='periodic_scan2cloud',
    #         parameters=[{
    #             'hz': 10.0,
    #             'duration': 0.2,
    #             'delay': 0.05,
    #             'tf_prefix': ''
    #         }],
    #         remappings=[('cloud', '/laser_cloud')]
    #     ),
    # ]

    dual_laser_merger_node = ComposableNodeContainer(
        name='demo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[
                    {'laser_1_topic': '/lidar1/scan'},
                    {'laser_2_topic': '/lidar2/scan'},
                    {'merged_scan_topic': '/merged'},
                    {'merged_cloud_topic': '/merged_cloud'},
                    {'target_frame': 'base_link'},
                    {'laser_1_x_offset': 0.0},
                    {'laser_1_y_offset': 0.0},
                    {'laser_1_yaw_offset': 0.0},
                    {'laser_2_x_offset': 0.0},
                    {'laser_2_y_offset': 0.0},
                    {'laser_2_yaw_offset': 0.0},
                    {'tolerance': 0.01},
                    {'queue_size': 5},
                    {'angle_increment': 0.001},
                    {'scan_time': 0.067},
                    {'range_min': 0.01},
                    {'range_max': 25.0},
                    {'min_height': -1.0},
                    {'max_height': 1.0},
                    {'angle_min': -3.141592654},
                    {'angle_max': 3.141592654},
                    {'inf_epsilon': 1.0},
                    {'use_inf': True},
                    {'allowed_radius': 0.45},
                    {'enable_shadow_filter': True},
                    {'enable_average_filter': True},
                    ],
            )
        ],
        output='screen',
    )

#    laser_to_pointcloud_node = Node(
#        package='laser_to_pointcloud',
#        executable='laser_to_pointcloud_node',
#        name='laser_to_pointcloud',
#    )

    # 安全限制器节点
    safety_limiter_node = Node(
        package='safety_limiter',
        executable='safe_limiter',
        name='safety_limiter',
        parameters=[{
            'odom': '/odom',
            'cloud': '/merged_cloud',
            'cmd_vel': LaunchConfiguration('cmd_vel_in'),
            'cmd_vel_out': LaunchConfiguration('cmd_vel_out'),
            'disable_command': 'disable_command',
            'frequency_hz': 10.0,
            'terrain_condition': '',
            'max_lin_margin': 0.1,
            'min_lin_margin': 0.5,
            'near_obs_margin': 0.5,
            'base_link_id': 'base_link',
            'max_acceleration_limit': LaunchConfiguration('max_linear_acceleration'),
            'max_angular_acceleration_limit': LaunchConfiguration('max_angular_acceleration'),
            'stop_acceleration': 1.5,
            'max_lin_vel': LaunchConfiguration('max_linear_velocity'),
            'max_ang_vel': LaunchConfiguration('max_angular_velocity'),
            'max_scale_vel': 1.0,
            'collision_time_window': 1.0,
            'dwa_steps': 10,
            'robot_radius': 0.35,
            'deacceleration_margin': 0.1,
            'limit_angular_vel': False,
            'display_path': False,
            'display_radius': True,
            'visualize_vel_vector': False,
            'speed_limiter_mode': False,
            'disabled_on_init': LaunchConfiguration('disable_limiter'),
            'limiter_mode_topic': 'limiter_mode'
        }],
        respawn=True
    )

    return LaunchDescription(
        declared_arguments +
        [
            dual_laser_merger_node,
            #laser_to_pointcloud_node,
            # tf_nodes +
            # urg_nodes +
            # laser_assembler_nodes + 
            safety_limiter_node
        ]
    ) 
