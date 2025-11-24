from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imx477_dual_camera',
            executable='imx477_dual_camera_node',
            name='imx477_dual_camera_node',
            output='screen',
            parameters=[{
                'full_width': 4000,
                'full_height': 3000,
                'frame_id': 'imx477'
            }]
        )
    ])
