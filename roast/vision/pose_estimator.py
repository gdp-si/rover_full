"""Depth estimation for the monocular camera."""
from typing import Dict

import numpy as np

from roast import RobotProfile
from roast.glogging import Logger
from roast.vision.utils import xywh_to_xywhn


class PoseFilter:
    """Moving average filter for estimated pose coordinates."""

    def __init__(self, filter_size=10):
        self._items = []
        self._item = 0
        self._filter_size = filter_size

    @property
    def filter_size(self):
        return self._filter_size

    def update(self, item):
        """Update the filter with the new item."""
        self._item = item
        self._items.append(item)
        if len(self._items) > self._filter_size:
            self._items.pop(0)

    def get(self):
        """Return the filtered item."""
        return sum(self._items) / len(self._items)


class DepthEstimator:
    """Simple depth estimation for the monocular camera using bounding box size approximation."""

    LOG = Logger(_module_name="Depth Estimator")

    # For four cameras setup
    BOUNDING_BOX_WIDTH = 0.06  # 0.18
    BOUNDING_BOX_HEIGHT = 0.10  # 0.26

    # Considering the horizontal plane
    CAMERA_OFFSET = 45.0  # degrees
    # alignment of world frame with camera frame
    FRAME_OFFSET = 90.0  # degrees
    PIXEL_TO_ANGLE = 0.1875  # degrees/pixel

    def __init__(self, debug=False):
        """Depth Estimator Initialization

        Args:
            debug (bool, optional): Add Debug information. Defaults to False.
        """
        self.debug = debug
        self.FIELD_OF_VIEW = self.get_fov_after_cropping()
        self.LOG.INFO("Depth Estimator Initialized.")

    def estimate(self, bounding_box: dict, image_size: tuple) -> float:
        """Estimate the depth of the object.

        Args:
            bounding_box (tuple): The bounding box of the object following `xywh` fornat.
            image_size (tuple): The size of the image following `width` and `height` format.

        Returns:
            float: The depth of the object in m.
        """

        std_dev = self.BOUNDING_BOX_WIDTH / self.BOUNDING_BOX_HEIGHT

        bbox = (
            bounding_box["xcenter"],
            bounding_box["ycenter"],
            bounding_box["width"],
            bounding_box["height"],
        )

        # Normalization with whhh gaussian format
        # Convert to xywhn format
        bbox = xywh_to_xywhn(bbox, image_size[1], image_size[0])
        # Normalization with xywhn format
        bbox = (
            bbox[0] / image_size[1],
            bbox[1] / image_size[0],
            bbox[2] / image_size[1],
            bbox[3] / image_size[1],
        )

        if std_dev <= bbox[2] / bbox[3]:
            depth = round(self.BOUNDING_BOX_WIDTH / bbox[2] * 100, 2)  # mm
        else:
            depth = round(self.BOUNDING_BOX_HEIGHT / bbox[3] * 100, 2)

        # Unknown metric
        return depth * 1e-5  # m

    def estimate_angle(self, bounding_box: dict, in_radians: bool = False) -> float:
        """Estimate the angle of the object.

        ## Theory:

         - This function uses the pixel to angle conversion to estimate the angle of the object.
         - The camera offset is 45 degrees for 90 FoV per camera.
         -If we are considering right frame first,then back,left,front in order,
                then we should be adding frame offset of 90 degrees.
        """

        # Considering the horizontal plane
        xcenter = bounding_box["xcenter"]

        result = np.mod(
            (self.FIELD_OF_VIEW - xcenter) * self.PIXEL_TO_ANGLE
            + self.CAMERA_OFFSET
            - self.FRAME_OFFSET,
            360,
        )

        if in_radians:
            return np.deg2rad(result)

        return result

    def get_fov_after_cropping(self) -> int:
        """Returns the field of view in pixels after cropping the image."""

        if not RobotProfile.OAKD_ARRAY_LIST:
            raise ValueError("No OAK-D cameras were specified in the RobotProfile.")

        if len(RobotProfile.OAKD_ARRAY_LIST) == 4:
            self.BOUNDING_BOX_WIDTH = 0.06
            self.BOUNDING_BOX_HEIGHT = 0.10
            return RobotProfile.FOV_AFTER_CROPPING * 4

        elif len(RobotProfile.OAKD_ARRAY_LIST) == 2:
            if not (
                "left_oakd_camera" in RobotProfile.OAKD_ARRAY_LIST
                or "right_oakd_camera" in RobotProfile.OAKD_ARRAY_LIST
            ):
                raise ValueError(
                    f"Invalid OAK-D camera list: {RobotProfile.OAKD_ARRAY_LIST}"
                )
            self.BOUNDING_BOX_WIDTH = 0.17
            self.BOUNDING_BOX_HEIGHT = 0.21

            return RobotProfile.FOV_AFTER_CROPPING * 2

        elif len(RobotProfile.OAKD_ARRAY_LIST) == 3:
            assert (
                "back_oakd_camera" in RobotProfile.OAKD_ARRAY_LIST
            ), "Invalid OAK-D camera list: {RobotProfile.OAKD_ARRAY_LIST}"
            self.BOUNDING_BOX_WIDTH = 0.10
            self.BOUNDING_BOX_HEIGHT = 0.14

            return RobotProfile.FOV_AFTER_CROPPING * 3

        else:
            assert (
                "front_oakd_camera" in RobotProfile.OAKD_ARRAY_LIST
            ), "Invalid OAK-D camera list: {RobotProfile.OAKD_ARRAY_LIST}"
            self.BOUNDING_BOX_WIDTH = 0.25
            self.BOUNDING_BOX_HEIGHT = 0.30

            return RobotProfile.FOV_AFTER_CROPPING


class PoseEstimator(DepthEstimator):
    """Simple Pose Estimator"""

    LOG = Logger(_module_name="Pose Estimator")

    def __init__(self, filtered: bool = True, debug=False):
        """Pose Estimator Initialization

        Args:
            debug (bool, optional): Add Debug information. Defaults to False.
        """
        super().__init__(debug)
        self._filtered = filtered

        if self._filtered:
            # Apply moving average filter
            self.x_filter = PoseFilter(filter_size=7)
            self.y_filter = PoseFilter(filter_size=7)
        self.LOG.INFO("Pose Estimator Initialized.")

    def pose_estimate(self, result: dict, detections: list) -> Dict[str, float]:
        """Estimate the pose of the object.

        Args:
            result (dict): The bounding box of the object following.
            detections (list): The yolo detection result.

        Returns:
            dict: The pose of the object.
        """
        image_size = result["image_size"]
        if not result["person_mapping"]:
            self.LOG.INFO(
                "Threat Pose cannot be estimated without head and person detected together."
            )
            x = 0.0
            y = 0.0
            angle = 0.0
        else:
            # TODO: Handle multiple detections
            map_result = result["person_mapping"]
            # Compute the depth and angle of the object in the image
            head_idx = map_result.head
            head_bbox = detections[0].boxes[head_idx].xywh.tolist()[0]
            head_box = {
                "xcenter": head_bbox[0],
                "ycenter": head_bbox[1],
                "width": head_bbox[2],
                "height": head_bbox[3],
            }

            person_idx = map_result.person
            person_bbox = detections[0].boxes[person_idx].xywh.tolist()[0]
            person_box = {
                "xcenter": person_bbox[0],
                "ycenter": person_bbox[1],
                "width": person_bbox[2],
                "height": person_bbox[3],
            }

            depth = self.estimate(head_box, image_size)
            angle = self.estimate_angle(person_box)
            angle = np.deg2rad(angle)

            x = depth * np.cos(angle)
            y = depth * np.sin(angle)

            if self._filtered:
                self.x_filter.update(x)
                self.y_filter.update(y)
                x = self.x_filter.get()
                y = self.y_filter.get()

            result["depth"] = depth

        return {
            "x": round(x, 2),
            "y": round(y, 2),
            "theta": round(angle, 2),
        }
