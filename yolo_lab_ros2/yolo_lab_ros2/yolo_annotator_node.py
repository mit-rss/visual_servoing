"""
Instructor-provided ROS2 node.

- Subscribes to a camera Image topic
- Runs Ultralytics YOLO on each frame
- Converts YOLO results -> a simple Detection list
- Calls student code to filter + draw
- Publishes an annotated Image topic
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
from ultralytics import YOLO

from yolo_lab_ros2.student_yolo import (
    Detection, draw_detections, filter_detections, get_allowed_class_names)


class YoloAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_annotator")

        # ---- Parameters
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("annotated_topic", "/yolo/annotated_image")
        self.declare_parameter("model", "yolo11n.pt")
        self.declare_parameter("conf_threshold", 0.5)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        self.model_name = self.get_parameter("model").get_parameter_value().string_value
        self.conf_threshold = float(self.get_parameter("conf_threshold").get_parameter_value().double_value)

        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Publishing annotated images to: {self.annotated_topic}")
        self.get_logger().info(f"YOLO model: {self.model_name}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")

        self.model = YOLO(self.model_name)
        self.class_names = self.model.names

        allowed = get_allowed_class_names()
        self.get_logger().info(f"You've chosen to keep these classes: {allowed}")

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.pub = self.create_publisher(Image, self.annotated_topic, 10)

    def on_image(self, msg: Image) -> None:
        # Convert ROS -> OpenCV (BGR)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO inference
        try:
            results = self.model.predict(source=bgr, verbose=False)
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        if not results:
            return

        dets = self.results_to_detections(results[0])
        allowed = get_allowed_class_names()
        dets = filter_detections(dets, conf_threshold=self.conf_threshold, allowed=allowed)

        annotated = draw_detections(bgr, dets)

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    def results_to_detections(self, result):
        """
        Convert an Ultralytics result into our simple Detection list.

        Ultralytics v8:
          result.boxes.xyxy: (N, 4) tensor
          result.boxes.conf: (N,) tensor
          result.boxes.cls:  (N,) tensor
        """
        detections = []

        if result.boxes is None:
            return detections

        xyxy = result.boxes.xyxy
        conf = result.boxes.conf
        cls = result.boxes.cls

        # Convert Torch tensors -> CPU numpy
        xyxy_np = xyxy.detach().cpu().numpy() if hasattr(xyxy, "detach") else np.asarray(xyxy)
        conf_np = conf.detach().cpu().numpy() if hasattr(conf, "detach") else np.asarray(conf)
        cls_np = cls.detach().cpu().numpy() if hasattr(cls, "detach") else np.asarray(cls)

        for (x1, y1, x2, y2), c, k in zip(xyxy_np, conf_np, cls_np):
            class_id = int(k)
            class_name = self._class_names.get(class_id, str(class_id))
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(c),
                    x1=int(round(x1)),
                    y1=int(round(y1)),
                    x2=int(round(x2)),
                    y2=int(round(y2)),
                )
            )
        return detections


def main() -> None:
    rclpy.init()
    node = YoloAnnotatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
