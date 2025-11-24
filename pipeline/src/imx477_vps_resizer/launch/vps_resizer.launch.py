from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imx477_vps_resizer',
            executable='vps_resizer_node',
            name='imx477_vps_resizer_node',
            output='screen',
            parameters=[
                {
                    'input_width': 4000,
                    'input_height': 3000,
                    'output_width': 1920,
                    'output_height': 1080,
                    'input_topic': '/imx477/image_raw_4k',
                    'output_topic': '/imx477/image_raw_1080',
                }
            ]
        )
    ])
