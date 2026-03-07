#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import torch

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dataclasses import dataclass
from rclpy.node import Node
from typing import List, Set
from ultralytics import YOLO


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    # Bounding box coordinates in the original image:
    x1: int
    y1: int
    x2: int
    y2: int


class YoloAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_annotator")

        # Declare and get ROS parameters
        self.model_name = (
            self.declare_parameter("model", "yolo11n.pt")
            .get_parameter_value()
            .string_value
        )
        self.conf_threshold = (
            self.declare_parameter("conf_threshold", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.iou_threshold = (
            self.declare_parameter("iou_threshold", 0.7)
            .get_parameter_value()
            .double_value
        )

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(self.model_name)
        self.model.to(self.device)

        self.allowed_cls = [
            i for i, name in self.model.names.items()
            if name in self.get_allowed_class_names()
        ]

        self.get_logger().info(f"Running {self.model_name} on device {self.device}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")
        if self.allowed_cls:
            self.get_logger().info(f"You've chosen to keep these class IDs: {self.allowed_cls}")
        else:
            self.get_logger().warn("No allowed classes matched the model's class list.")

        # Create publisher and subscribers
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.on_image, 10)
        self.pub = self.create_publisher(
            Image, "/yolo/annotated_image", 10)

    def get_allowed_class_names(self) -> Set[str]:
        """
        Return the set of COCO class names you want to keep.
        examples: "chair", "couch", "tv", "laptop", "dining table",...
        """
        # TODO: Customize this set for the lab
        return {"chair", "dining table"}

    def on_image(self, msg: Image) -> None:
        # Convert ROS -> OpenCV (BGR)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO inference
        try:
            results = self.model(
                bgr,
                classes=self.allowed_cls,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        if not results:
            return

        # Convert results to Detection List
        dets = self.results_to_detections(results[0])

        # Draw detections on BGR image
        annotated = self.draw_detections(bgr, dets)

        # Publish annotated BGR image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    def results_to_detections(self, result):
        """
        Convert an Ultralytics result into a Detection list.

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
            class_name = self.model.names.get(class_id, str(class_id))
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=float(c),
                    x1=round(x1),
                    y1=round(y1),
                    x2=round(x2),
                    y2=round(y2),
                )
            )
        return detections

    def draw_detections(
        self,
        bgr_image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:

        out_image_copy = bgr_image.copy()

        for det in detections:
            pass

            # TODO: Get a bounding box for the detection

            # TODO: Label the box with the class name and confidence

            # TODO: Put the bounding box and label on the output image.
            #       Make sure to color the bounding boxes for different
            #       classes using different colors

        return out_image_copy


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
