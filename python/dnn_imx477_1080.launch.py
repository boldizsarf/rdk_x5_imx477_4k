from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dnn_example_config_file = LaunchConfiguration("dnn_example_config_file")
    dnn_example_image_width = LaunchConfiguration("dnn_example_image_width")
    dnn_example_image_height = LaunchConfiguration("dnn_example_image_height")

    return LaunchDescription([
        DeclareLaunchArgument(
            "dnn_example_config_file",
            default_value="/opt/tros/humble/lib/dnn_node_example/config/yolov11workconfig.json",
            description="Path to DNN config file",
        ),
        DeclareLaunchArgument(
            "dnn_example_image_width",
            default_value="1920",
            description="Input image width",
        ),
        DeclareLaunchArgument(
            "dnn_example_image_height",
            default_value="1080",
            description="Input image height",
        ),

        Node(
            package="hobot_codec",
            executable="hobot_codec_republish",
            name="codec_1080_nv12_to_jpeg",
            output="screen",
            parameters=[
                {"channel": 0},
                {"in_mode": "ros"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/imx477/image_raw_1080"},
                {"pub_topic": "/image_jpeg"},
                {"jpg_quality": 70.0},
                {"input_framerate": 30},
                {"output_framerate": 0},
                {"dump_output": False},
                {"dump_frame_count": -1},
            ],
        ),

        Node(
            package="hobot_codec",
            executable="hobot_codec_republish",
            name="codec_4k_nv12_to_jpeg",
            output="screen",
            parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "nv12"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"sub_topic": "/imx477/image_raw_4k"},
                {"pub_topic": "/image_jpeg_4k"},
                {"jpg_quality": 70.0},
                {"input_framerate": 30},
                {"output_framerate": 0},
                {"dump_output": False},
                {"dump_frame_count": -1},
            ],
        ),

        Node(
            package="dnn_node_example",
            executable="example",
            name="dnn_node_example_imx477_1080",
            output="screen",
            parameters=[
                {"feed_type": 1},
                {"image": "config/test.jpg"},
                {"image_type": 1},
                {"dump_render_img": 0},
                {"is_shared_mem_sub": 0},
                {"config_file": dnn_example_config_file},
                {"msg_pub_topic_name": "hobot_dnn_detection"},
                {"info_msg_pub_topic_name": "hobot_dnn_detection_info"},
                {"ros_img_topic_name": "/imx477/image_raw_1080"},
                {"sharedmem_img_topic_name": "/hbmem_img"},
            ],
        ),

        Node(
            package="websocket",
            executable="websocket",
            name="websocket",
            output="screen",
            parameters=[
                {"image_topic": "/image_jpeg_4k"},
                {"image_type": "mjpeg"},
                {"only_show_image": False},
                {"output_fps": 0},
                {"channel": 0},
                {"smart_topic": "/hobot_dnn_detection"},
            ],
            arguments=["--ros-args", "--log-level", "info"],
        ),
    ])
