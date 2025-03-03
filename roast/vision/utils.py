"""Util functions for vision module."""
import math


def xywh_center_to_xyxy(box):
    """Convert bounding box format from xcenter, ycenter, width, height to xyxy."""
    x1, y1, w, h = box
    result = [x1 - w / 2, y1 - h / 2, x1 + w / 2, y1 + h / 2]

    return int(result[0]), int(result[1]), int(result[2]), int(result[3])


def xyxy_to_xywh(box):
    """Convert bounding box format from xyxy to xywh."""
    x1, y1, x2, y2 = box
    return int((x1 + x2) / 2), int((y1 + y2) / 2), int(x2 - x1), int(y2 - y1)


def check_bb_inside_bb(bb1, bb2, threshold=0.9):
    """Check the bounding box bb1 is inside bb2 within a threshold.
    Bounding box format is xyxy.
    """
    x1, y1, x2, y2 = bb1
    x3, y3, x4, y4 = bb2
    bb1_area = (x2 - x1) * (y2 - y1)
    intersection_area = max(0, min(x2, x4) - max(x1, x3)) * max(
        0, min(y2, y4) - max(y1, y3)
    )
    if bb1_area == 0:
        return False

    return intersection_area / bb1_area >= threshold


def xywh_to_xywhn(box, img_width, img_height):
    """Convert bounding box format from xywh to xywhn."""
    x, y, w, h = box
    return x / img_width, y / img_height, w / img_width, h / img_height


def get_centroid(box):
    """Get centroid of bounding box."""
    x1, y1, x2, y2 = box
    return (x1 + x2) / 2, (y1 + y2) / 2


def get_distance(box1, box2):
    """Get distance between two bounding boxes."""
    x1, y1 = get_centroid(box1)
    x2, y2 = get_centroid(box2)
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
