import os
from typing import Any, Dict, List, NamedTuple, Tuple, Union

import cv2
import numpy as np
import torch

from roast.glogging import Logger
from roast.utils import euclidean_distance
from roast.vision import ObjectDetection, PoseEstimator
from roast.vision.configs import ObjectDetectionClass, ThreatDetectionParameters
from roast.vision.utils import check_bb_inside_bb, xywh_center_to_xyxy

PersonMapping = NamedTuple("PersonMapping", [("person", int), ("head", int)])
WeaponMapping = NamedTuple("WeaponMapping", [("weapon", int), ("person", int)])


class ThreatDetection(ObjectDetection):
    """Object tracking class."""

    LOG = Logger(_module_name="Threat Detection")

    def __init__(self, use_gpu=True, debug=False):
        super().__init__(use_gpu=use_gpu, debug=debug, output_info="xywh")

        # Update the configurations for object detection
        self._update_model_configs()

        if debug:
            self._visualize_detection = ThreatDetectionParameters.visualize_detection

    def _update_model_configs(self):
        """Update configs."""
        self.model.conf_thres = ThreatDetectionParameters.confidence_threshold
        self.model.iou_thres = ThreatDetectionParameters.iou_threshold
        self.model.max_det = ThreatDetectionParameters.max_detections
        self._weapon_classes = ThreatDetectionParameters.weapon_classes
        self._classes = ThreatDetectionParameters.classes

    def _weapon_detected(self, detected_classes: List[str]) -> bool:
        for weapon in self._weapon_classes:
            if weapon.name.lower() in detected_classes:
                return True

        return False

    def detect_target(
        self, frame: np.ndarray, get_detections: bool = False
    ) -> Union[List[Dict[str, Any]], Tuple[List[Dict[str, Any]], list]]:
        """Detect target in frame.

        -------
        ### Theory:

         - Detect the person and weapon in the frame.
         - If the person and weapon are detected, check if they overlap.
         - If they overlap, return the information of the person.
        """
        threat_detections: List[Dict[str, Any]] = []

        # Detect objects in frame
        detections = self.detect(frame, format_result=False)

        detected_classes = []
        if detections:
            detected_classes = detections[0].boxes.cls.tolist()
            detected_classes = [
                detections[0].names[int(cls)] for cls in detected_classes
            ]

        # Check if any weapon classes are detected
        if not self._weapon_detected(detected_classes):
            return threat_detections, detections

        # Get the index of the weapon, heads and person
        weapon_idxs = self._get_idx(self._weapon_classes, detected_classes)
        person_idxs = self._get_idx("person", detected_classes)
        head_idxs = self._get_idx("head", detected_classes)

        # Map head to person
        mapped_ids = self._map_head_to_person(head_idxs, person_idxs, detections)

        if self.debug:
            print("*" * 50)
            print(f"detected_classes: {detected_classes}")
            print(f"weapon_classes: {self._weapon_classes}")
            print(f"weapon_detected: {self._weapon_detected(detected_classes)}")
            print(f"weapon_idxs: {weapon_idxs}")
            print(f"person_idxs: {person_idxs}")
            print(f"head_idxs: {head_idxs}")
            print(f"person_mapping: {mapped_ids}")
            print("*" * 50)

        if self.debug and self._visualize_detection:
            self._debug_result(frame, detections)

        threat_detections = self._map_weapon_to_person(
            weapon_idxs,
            person_idxs,
            detections,
            frame_size=frame.shape,
            mapped_ids=mapped_ids,
        )

        if get_detections:
            return threat_detections, detections

        return threat_detections

    def _map_weapon_to_person(
        self,
        weapon_idxs: Union[List[int], int],
        person_idxs: Union[List[int], int],
        detections: List,
        **kwargs,
    ) -> List[Dict[str, Any]]:
        threat_detections = []
        frame_size = kwargs.get("frame_size")
        person_mapping = kwargs.get("mapped_ids")

        # TODO: Remove once multiple threat detection is supported
        if isinstance(weapon_idxs, int):
            weapon_idxs = [weapon_idxs]
        if isinstance(person_idxs, int):
            person_idxs = [person_idxs]

        for weapon_idx in weapon_idxs:
            for person_idx in person_idxs:
                if self._detect_overlap(weapon_idx, person_idx, detections):
                    # Get result
                    result = detections[0]

                    boxes = result[person_idx].boxes.xywh.tolist()[0]
                    class_name = result[person_idx].boxes.cls.tolist()
                    class_name = result.names[int(class_name[0])]
                    confidence = result[person_idx].boxes.conf[0]
                    weapon_map = WeaponMapping(weapon=weapon_idx, person=person_idx)
                    person_map = (
                        person_mapping[person_idx]  # type: ignore
                        if person_idx in person_mapping  # type: ignore
                        else {}
                    )
                    threat_detections.append(
                        {
                            "xcenter": int(boxes[0]),
                            "ycenter": int(boxes[1]),
                            "width": int(boxes[2]),
                            "height": int(boxes[3]),
                            "image_size": frame_size,
                            "class": class_name,
                            "confidence": float(confidence),
                            "name": "threat",
                            "person_mapping": person_map,
                            "weapon_mapping": weapon_map,
                        }
                    )

        return threat_detections

    def _map_head_to_person(
        self,
        head_idxs: Union[List[int], int],
        person_idxs: Union[List[int], int],
        detections: List,
    ) -> dict:
        mapped_idxs: Dict[int, PersonMapping] = {}

        # TODO: Remove once multiple threat detection is supported
        if isinstance(head_idxs, int):
            head_idxs = [head_idxs]
        if isinstance(person_idxs, int):
            person_idxs = [person_idxs]

        person_mapping = []
        for head_idx in head_idxs:
            for person_idx in person_idxs:
                # Check if the bounding box of the head is inside the bounding box of the person
                bbox_head = detections[0][head_idx].boxes.xyxy.tolist()[0]
                bbox_person = detections[0][person_idx].boxes.xyxy.tolist()[0]

                if check_bb_inside_bb(bbox_head, bbox_person):
                    person_mapping.append(
                        PersonMapping(person=person_idx, head=head_idx)
                    )

        # Check if the mapped indexes are unique and if not remove the duplicates
        for person_map in person_mapping:
            if person_map.person not in mapped_idxs:
                mapped_idxs[person_map.person] = person_map
            else:
                # Check if the (xmin, ymin) of the head is near the (xmin, ymin) of the person
                reference_bbox = detections[0][person_map.person].boxes.xyxy.tolist()[0]
                head_bbox = detections[0][person_map.head].boxes.xyxy.tolist()[0]
                head2_bbox = detections[0][
                    mapped_idxs[person_map.person].head
                ].boxes.xyxy.tolist()[0]

                result_bbox = self._estimate_near_bbox(
                    reference_bbox, head_bbox, head2_bbox
                )
                if result_bbox == head_bbox:
                    mapped_idxs[person_map.person] = person_map
                else:
                    mapped_idxs[person_map.person] = mapped_idxs[person_map.person]

        return mapped_idxs

    @staticmethod
    def _estimate_near_bbox(reference_bbox: list, bbox1: list, bbox2: list):
        """Estimate which bbox is near the reference bbox."""
        xmin_ref, ymin_ref, _, _ = reference_bbox
        xmin_bbox1, ymin_bbox1, _, _ = bbox1
        xmin_bbox2, ymin_bbox2, _, _ = bbox2

        # Calculate the distance between the reference bbox and bbox1
        distance_bbox1 = euclidean_distance(
            (xmin_ref, ymin_ref), (xmin_bbox1, ymin_bbox1)
        )
        distance_bbox2 = euclidean_distance(
            (xmin_ref, ymin_ref), (xmin_bbox2, ymin_bbox2)
        )

        if distance_bbox1 < distance_bbox2:
            return bbox1

        return bbox2

    def _get_idx(
        self, class_name: Union[str, List[ObjectDetectionClass]], detected_classes
    ) -> Union[int, List[int]]:
        if isinstance(class_name, str):
            return [
                idx for idx, name in enumerate(detected_classes) if name == class_name
            ]
        elif isinstance(class_name, list):
            result = []
            for name in class_name:
                ids = [
                    idx
                    for idx, detected_name in enumerate(detected_classes)
                    if detected_name in [name.name.lower(), name.name.upper()]
                ]
                result.extend(ids)
            return result
        else:
            raise ValueError("Invalid class_name type.")

    def _detect_overlap(self, weapon_idx: int, person_idx: int, detections) -> bool:
        """Detect if two bounding boxes overlap."""

        result = detections[0]
        # Get the bounding box of the weapon
        boxes = result[weapon_idx].boxes.xywh.tolist()[0]
        weapon_xcenter = boxes[0]
        weapon_ycenter = boxes[1]
        weapon_width = boxes[2]
        weapon_height = boxes[3]

        # Get the bounding box of the person
        boxes = result[person_idx].boxes.xywh.tolist()[0]
        person_xcenter = boxes[0]
        person_ycenter = boxes[1]
        person_width = boxes[2]
        person_height = boxes[3]

        # Check if the weapon and person overlap
        if (
            weapon_xcenter - weapon_width / 2 < person_xcenter + person_width / 2
            and weapon_xcenter + weapon_width / 2 > person_xcenter - person_width / 2
            and weapon_ycenter - weapon_height / 2 < person_ycenter + person_height / 2
            and weapon_ycenter + weapon_height / 2 > person_ycenter - person_height / 2
        ):
            return True

        return False

    def _debug_result(self, frame, detections):
        if not os.getenv("DISPLAY"):
            return

        image = frame.copy()

        for result in detections:
            for idx, box in enumerate(result.boxes):
                xmin, ymin, xmax, ymax = box.xyxy.tolist()[0]
                xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
                cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(
                    image,
                    result.names[int(result.boxes.cls[idx])],
                    (xmin, ymin - 5),
                    0,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        cv2.namedWindow("Threat Detection", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Threat Detection", image)
        cv2.waitKey(1)

    def __del__(self):
        del self.model
        torch.cuda.empty_cache()
        if self.debug:
            cv2.destroyWindow("Threat Detection")


class ThreatTracking:
    """Person tracking class."""

    LOG = Logger(_module_name="Threat Tracking")

    def __init__(self, debug: bool = False):
        """Threat Tracking Initialization"""
        self._object_detection = ThreatDetection(debug=debug)
        self._pose_estimator = PoseEstimator(
            filtered=ThreatDetectionParameters.apply_filter,
            debug=debug,
        )

        # Get threat detection parameters
        self._apply_safety_perimeter = ThreatDetectionParameters.apply_safety_perimeter
        if self._apply_safety_perimeter:
            self._safety_perimeter_radius = (
                ThreatDetectionParameters.safety_perimeter_radius
            )

        self._debug = debug

        if self._debug:
            self._visualize_tracking = ThreatDetectionParameters.visualize_tracking
            self._visualize_weapon = ThreatDetectionParameters.visualize_weapon

    def track(
        self, image: cv2.Mat, format_result: bool = True, get_image: bool = False
    ) -> Union[
        List[Dict[str, Any]],
        Tuple[Union[Dict[str, Any], List[Dict[str, Any]]], cv2.Mat],
        Dict[str, Any],
    ]:
        """Track the threat in the image.

        Args:
            image (str): The image path.
            format_result (bool, optional): Format the result. Defaults to True.

        Returns:
            dict: The tracking result.
        """

        # Check if image height matches the model input height
        assert (
            image.shape[0] == 400
        ), "Image height does not match the model input height."

        # Detect the threat
        detection_result, detections = self._object_detection.detect_target(
            image, get_detections=True
        )

        if isinstance(detection_result, dict):
            detection_result = [detection_result]

        # Estimate the pose
        for result in detection_result:
            result["pose"] = self._pose_estimator.pose_estimate(result, detections)  # type: ignore

        if self._debug:
            self._debug_result(image, detection_result, detections)

        if get_image and detection_result:
            for result in detection_result:
                image = self._draw_result(image, result)
        # Process the result
        if format_result and detection_result:
            # TODO: Handle multiple detections
            detection_result = self._format_result(detection_result[0])
        elif not detection_result:
            detection_result = {
                "is_threat_detected": False,
                "near_safety_perimeter": False,
                "pose": {"x": 0, "y": 0, "theta": 0},
                "threat_angle": 0,
            }

        if get_image:
            return detection_result, image

        return detection_result

    def _draw_result(self, image: cv2.Mat, detection_result: Dict[str, Any]) -> cv2.Mat:
        """Draw bounding box aroung the image for the threat."""
        x, y, w, h = (
            detection_result["xcenter"],
            detection_result["ycenter"],
            detection_result["width"],
            detection_result["height"],
        )
        xmin, ymin, xmax, ymax = xywh_center_to_xyxy(box=(x, y, w, h))
        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
        cv2.putText(
            image,
            detection_result["name"],
            (xmin, ymin - 5),
            0,
            0.5,
            (0, 255, 0),
            2,
        )

        return image

    def _format_result(self, detection_result):
        result = {}
        if detection_result["pose"]["x"] == 0 and detection_result["pose"]["y"] == 0:
            result["is_threat_detected"] = False
        else:
            result["is_threat_detected"] = True

        if self._apply_safety_perimeter:
            result["pose"], near_safety_perimeter = self.get_safety_coordinate(
                detection_result["pose"]
            )
            result["threat_angle"] = detection_result["pose"]["theta"]
            result["near_safety_perimeter"] = near_safety_perimeter

            if near_safety_perimeter:
                # Reset the pose
                result["pose"] = {"x": 0, "y": 0, "theta": 0}
        else:
            result["pose"] = detection_result["pose"]
            result["threat_angle"] = detection_result["pose"]["theta"]
            result["near_safety_perimeter"] = False

        return result

    def get_safety_coordinate(self, detection_pose: dict):
        """Apply safety perimeter given the pose of the person and radius of the perimeter."""
        x, y = detection_pose["x"], detection_pose["y"]

        # Calculate the vector distance between the camera and the threat
        distance = np.sqrt(x**2 + y**2)

        # New depth value
        distance_1 = distance - self._safety_perimeter_radius
        theta = detection_pose["theta"]

        if (
            distance_1
            <= self._safety_perimeter_radius + ThreatDetectionParameters.error_tolerance
        ):
            self.LOG.INFO("Robot inside safety perimeter")
            # Assume the safety perimeter as distance to keep a safe distance
            detection_pose["x"] = self._safety_perimeter_radius * np.cos(theta)
            detection_pose["y"] = self._safety_perimeter_radius * np.sin(theta)

            return detection_pose, True  # Near safety perimeter

        detection_pose["x"] = distance_1 * np.cos(theta)
        detection_pose["y"] = distance_1 * np.sin(theta)

        return detection_pose, False  # Not near safety perimeter

    def _debug_result(self, image, detection_result, detections):
        if not os.getenv("DISPLAY"):
            return

        for result in detection_result:
            x, y, w, h = (
                result["xcenter"],
                result["ycenter"],
                result["width"],
                result["height"],
            )

            if result["weapon_mapping"] and self._visualize_weapon:
                map_result = result["weapon_mapping"]
                weapon_idx = map_result.weapon
                x_weapon, y_weapon, w_weapon, h_weapon = (
                    detections[0].boxes[weapon_idx].xywh.tolist()[0]
                )
                (
                    xmin_weapon,
                    ymin_weapon,
                    xmax_weapon,
                    ymax_weapon,
                ) = xywh_center_to_xyxy(box=(x_weapon, y_weapon, w_weapon, h_weapon))
                cv2.rectangle(
                    image,
                    (xmin_weapon, ymin_weapon),
                    (xmax_weapon, ymax_weapon),
                    (0, 255, 0),
                    2,
                )

            # Draw the bounding box
            if self._visualize_tracking:
                xmin, ymin, xmax, ymax = xywh_center_to_xyxy(box=(x, y, w, h))
                cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(
                    image,
                    result["name"],
                    (xmin, ymin - 5),
                    0,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        cv2.namedWindow("Threat Tracking", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("Threat Tracking", image)
        cv2.waitKey(1)

    def __del__(self):
        if self._debug:
            cv2.destroyWindow("Threat Tracking")
