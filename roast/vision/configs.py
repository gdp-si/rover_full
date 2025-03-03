"""Vision configurations."""
from enum import Enum, auto
from typing import NamedTuple

Range = NamedTuple("Range", [("min", float), ("max", float)])


class ObjectDetectionClass(Enum):
    """Object detection classes."""

    PISTOL = 0
    RIFLE = auto()
    FIRE = auto()
    PERSON = auto()
    KNIFE = auto()
    HEAD = auto()
    MOBILE = auto()


class ObjectDetectionParameters:
    """Parameters for object detection."""

    confidence_threshold = 0.60
    device = 0
    iou_threshold = 0.45
    agnostic_nms = True
    classes = [
        ObjectDetectionClass.PISTOL,
        ObjectDetectionClass.RIFLE,
        ObjectDetectionClass.FIRE,
        ObjectDetectionClass.PERSON,
        ObjectDetectionClass.KNIFE,
        ObjectDetectionClass.HEAD,
        ObjectDetectionClass.MOBILE,
    ]
    max_detections = 100


class ThreatDetectionParameters(ObjectDetectionParameters):
    """Parameters for threat detection."""

    weapon_classes = [
        ObjectDetectionClass.PISTOL,
        ObjectDetectionClass.RIFLE,
        ObjectDetectionClass.KNIFE,
    ]
    apply_filter = True
    apply_safety_perimeter = True

    # Safety perimeter = robot footprint width + depth error threshold + inflation radius
    # Safety_perimeter = 0.5 + 1.0 + 0.4
    safety_perimeter_radius = (
        0.7  # Considering inflation radius + minimal safety for robot
    )
    error_tolerance = 0.0

    # Debug
    visualize_detection = True
    visualize_tracking = True
    visualize_weapon = False


class PeopleMassDetectionParameters(ObjectDetectionParameters):
    """Parameters for the people mass detection."""

    interested_depth = Range(min=1.0, max=4.0)
    interested_angle = Range(min=-0.5, max=0.5)

    # Depth tolerance between people in the same group
    depth_tolerance = 0.5

    num_groups = 1  # Number of groups
    num_people = 4  # Number of people in each group


class COCODataset:
    labels = [
        "person",
        "bicycle",
        "car",
        "motorbike",
        "aeroplane",
        "bus",
        "train",
        "truck",
        "boat",
        "traffic light",
        "fire hydrant",
        "stop sign",
        "parking meter",
        "bench",
        "bird",
        "cat",
        "dog",
        "horse",
        "sheep",
        "cow",
        "elephant",
        "bear",
        "zebra",
        "giraffe",
        "backpack",
        "umbrella",
        "handbag",
        "tie",
        "suitcase",
        "frisbee",
        "skis",
        "snowboard",
        "sports ball",
        "kite",
        "baseball bat",
        "baseball glove",
        "skateboard",
        "surfboard",
        "tennis racket",
        "bottle",
        "wine glass",
        "cup",
        "fork",
        "knife",
        "spoon",
        "bowl",
        "banana",
        "apple",
        "sandwich",
        "orange",
        "broccoli",
        "carrot",
        "hot dog",
        "pizza",
        "donut",
        "cake",
        "chair",
        "sofa",
        "pottedplant",
        "bed",
        "diningtable",
        "toilet",
        "tvmonitor",
        "laptop",
        "mouse",
        "remote",
        "keyboard",
        "cell phone",
        "microwave",
        "oven",
        "toaster",
        "sink",
        "refrigerator",
        "book",
        "clock",
        "vase",
        "scissors",
        "teddy bear",
        "hair drier",
        "toothbrush",
    ]


class CustomDataset:
    labels = ["pistol", "rifle", "fire", "person", "knife", "head", "mobile"]
