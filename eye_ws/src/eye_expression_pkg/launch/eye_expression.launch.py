import launch
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        


        
        # 启动 EyeExpressionNode 节点
        Node(
            package='eye_expression_pkg',
            executable='eye_expression_node',
            name='eye_expression_node',
            output='screen',
            parameters=[],
            remappings=[],
            namespace='',
            # 可以添加其他配置
        ),
    ])
