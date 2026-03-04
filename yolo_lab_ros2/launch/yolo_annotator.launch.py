from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="yolo_lab_ros2",
                executable="yolo_annotator",
                name="yolo_annotator",
                output="screen",
                parameters=[
                    {
                        # ZED RGB topic
                        "image_topic": "/zed/zed_node/rgb/image_rect_color",
                        # Annotated output topic
                        "annotated_topic": "/yolo/annotated_image",
                        # Ultralytics model name or path:
                        "model": "yolo11n.pt",
                        "conf_threshold": 0.5,
                    }
                ],
            )
        ]
    )
