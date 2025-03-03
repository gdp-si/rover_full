"""People mass detection."""
from copy import deepcopy
from typing import Any, Dict, List

import cv2
import numpy as np
import torch

from roast.glogging import Logger
from roast.utils import approx
from roast.vision.configs import ObjectDetectionClass, PeopleMassDetectionParameters
from roast.vision.object_detection import ObjectDetection
from roast.vision.pose_estimator import PoseEstimator
from roast.vision.utils import check_bb_inside_bb


class PeopleMassDetection(ObjectDetection):
    """People mass detection."""

    LOG = Logger(_module_name=__name__)

    def __init__(self, use_gpu: bool = True, debug: bool = False):
        """Initialize PeopleMassDetection class."""
        super().__init__(use_gpu=use_gpu, debug=debug, output_info="xywh")
        self._pose_estimator = PoseEstimator(debug=debug)

        # Parameters
        self._configs = PeopleMassDetectionParameters

    def detect_people(self, frame: cv2.Mat):
        """Detect people in the frame."""

        detections = self.detect(frame, format_result=False)

        # Filter detections to only people and possibly heads
        people_detections = self._filter_detections(detections)

        # Format results to dictionaries
        people_detections = self._format_detections(people_detections)

        # Estimate depth and angle
        for detection in people_detections:
            detection["depth"] = self._pose_estimator.estimate(
                detection["bbox"], frame.shape
            )
            detection["angle"] = self._pose_estimator.estimate_angle(
                detection["bbox"], in_radians=True
            )

        # Group people
        groups = self._group_people(people_detections)

        return groups

    def _group_people(self, detections: List[dict]) -> Dict[int, Dict[str, Any]]:
        """Group people based on their depth and angle.

        ## Theory:

        1. Filter people based on depth.
        2. Group people based on their depth and angle.
        3. Calculate the center of the group.
        4. Calculate the position of the group.
        """
        groups = {}
        group_id = 0
        for detection in detections:
            depth = detection["depth"]
            angle = detection["angle"]
            groups[group_id] = {"people": [detection], "depth": depth, "angle": angle}

            for detection_2 in detections:
                if approx(
                    detection["depth"],
                    detection_2["depth"],
                    self._configs.depth_tolerance,
                ):
                    if detection_2 not in groups[group_id]["people"]:
                        groups[group_id]["people"].append(detection_2)

            group_id += 1

        # Calculate the center of the group by picking a person center to the group
        for group_id, group in groups.items():
            group["center"] = group["people"][0]["bbox"]

            # Calculate the position of the group
            if len(group["people"]) > 1:
                for detection in group["people"]:
                    if check_bb_inside_bb(detection["bbox"], group["center"]):
                        group["center"] = detection["bbox"]

        # Calculate the position of the group
        groups_copy = deepcopy(groups)
        for group_id, group in groups_copy.items():
            if len(group["people"]) >= self._configs.num_people:
                group["pose"] = {
                    "x": np.cos(group["angle"]) * group["depth"],
                    "y": np.sin(group["angle"]) * group["depth"],
                    "theta": group["angle"],
                }
            else:
                # Remove groups with less than the expected number of people
                del groups_copy[group_id]
        groups = groups_copy

        # Only return the first num_groups groups
        return dict(list(groups.items())[: self._configs.num_groups])

    def _format_detections(self, detections: List) -> List[dict]:
        """Format detections to a dictionary."""
        people_detections = []
        for detection in detections:
            bbox = {
                "xcenter": int(detection.xywh.tolist()[0][0]),
                "ycenter": int(detection.xywh.tolist()[0][1]),
                "width": int(detection.xywh.tolist()[0][2]),
                "height": int(detection.xywh.tolist()[0][3]),
            }
            people_detections.append(
                {
                    "bbox": bbox,
                    "confidence": float(detection.conf),
                    "depth": 1000.0,
                    "angle": 0.0,
                    "class": ObjectDetectionClass(int(detection.cls)).name,
                }
            )

        return people_detections

    def _filter_detections(self, detections: List) -> List:
        """Filter detections to only people."""
        people_detections = []

        if detections:
            for detection in detections[0].boxes:
                if int(detection.cls) == ObjectDetectionClass.PERSON.value:
                    people_detections.append(detection)

        return people_detections

    def __del__(self):
        del self.model
        torch.cuda.empty_cache()
        if self.debug:
            cv2.destroyWindow("People Mass Detection")


def main():
    """Main function for testing purpose"""

    cap = cv2.VideoCapture(0)
    detector = PeopleMassDetection(use_gpu=True, debug=False)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect_people(frame)
        print(detections)

        cv2.imshow("People Mass Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
