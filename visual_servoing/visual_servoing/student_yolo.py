"""
Learn how to use YOLO outputs:

- what classes exist (COCO names)
- how to filter by class + confidence
- how to interpret boxes (xyxy)
- how to draw annotated outputs
"""

from dataclasses import dataclass
from typing import List, Sequence, Set, Tuple

import cv2
import numpy as np


# -----------------------------
# Data model used by the wrapper
# -----------------------------

@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    # pixel coords in the original image:
    x1: int
    y1: int
    x2: int
    y2: int


def get_allowed_class_names() -> Set[str]:
    """
    Return the set of COCO class names you want to keep.
    examples: "chair", "couch", "tv", "laptop", "dining table",...

    TODO: Customize this set for the lab.
    """
    # Example default: chairs + tables
    return {"chair", "dining table"}


def should_keep_detection(det: Detection, conf_threshold: float, allowed: Set[str]) -> bool:
    """
    Decide whether to keep a detection.

    TODO: Adjust filtering logic
    - Keep only allowed classes
    - Keep only confidence > threshold
    - Keep boxes above a minimum area
    - Come up with your own logic!
    """
    raise NotImplementedError
    return False


def filter_detections(
    detections: Sequence[Detection],
    conf_threshold: float,
    allowed: Set[str],
) -> List[Detection]:
    """
    Filter detections based on student policy.
    - Sort by confidence by using your should_keep_detection function
    - Pick top-k per class
    """
    raise NotImplementedError
    kept = detections
    return kept


def class_color(class_name: str) -> Tuple[int, int, int]:
    """
    Return a BGR color for a given class name.

    TODO: Update colors and make a mapping based on the objects you picked
    """
    if class_name == "chair":
        return (255, 0, 0)
    if class_name == "dining table":
        return (0, 255, 0)
    else:
        raise NotImplementedError
    return


def draw_detections(
    bgr_image: np.ndarray,
    detections: Sequence[Detection],
) -> np.ndarray:
    """
    Draw bounding boxes and labels on the image.
    - Change label format
    - Draw center points
    - Highlight "best" detection
    """
    out_image_copy = bgr_image.copy()

    for det in detections:
        pass

        # TODO: Get a bounding box for the detection

        # TODO: Label the box with the class name and confidence

        # TODO: Put the bounding box and label on the output image, using class_color

    return out_image_copy
