"""Camera Calibration class for robot and CCTV cameras"""
import os

import cv2
import numpy as np
import yaml

from roast.glogging import Logger


class Calibration:
    """Calibration class for robot"""

    LOG = Logger(_module_name=os.path.basename(__file__))

    def __init__(self, output_path: str = "./config/camera_calibration.yaml"):
        self.output_path = os.path.join(os.path.dirname(__file__), output_path)
        self.calibration_data: dict = {}

    def load_calibration_model(self):
        """Load the existing calibration model

        Raises:
            FileExistsError: If the calibration model does not exist
        """

        if os.path.isfile(os.path.abspath(self.output_path)):
            self.LOG.INFO(
                f"Loading calibration model from {os.path.abspath(self.output_path)}"
            )
        else:
            raise FileExistsError("Calibration model not found")

        with open(self.output_path, encoding="utf-8") as fr:
            self.calibration_data = yaml.load(fr, Loader=yaml.FullLoader)

    def get_calibration_model(self):
        """Returns the camera calibration model

        Raises:
            Exception: If the calibration model does not exist

        Returns:
            dict: Calibration model
        """

        if self.calibration_data is not None:
            return self.calibration_data
        # else:
        if self.calibration_data is None:
            try:
                self.load_calibration_model()
                return self.calibration_data
            except FileExistsError as e:
                raise e

    def undistort_image(self, image: np.ndarray):
        """Returns the calibrated image

        Args:
            image (np.ndarray): Image to be calibrated

        Returns:
            np.ndarray: Calibrated image
        """

        if self.calibration_data is None:
            self.calibration_data = self.get_calibration_model()

        image = cv2.undistort(
            image,
            np.array(self.calibration_data["camera_matrix"]),
            np.array(self.calibration_data["dist_coefs"]),
            newCameraMatrix=np.array(self.calibration_data["camera_matrix"]),
        )

        return image
