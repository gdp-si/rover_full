import math
import os
from threading import Lock

import cv2
import depthai as dai
import numpy as np

from roast.glogging import Logger
from roast.io.configs import OAKDConfigs
from roast.settings import ROOT_DIR
from roast.vision.configs import CustomDataset, ThreatDetectionParameters


class Rectangle:
    """Check the overlap between two Bounding Box"""

    def __init__(self, bottom_left, top_right, colour):
        self.bottom_left = bottom_left
        self.top_right = top_right
        self.colour = colour

    def intersects(self, other):
        """intersection between 2 rectangles"""
        return not (
            self.top_right.x < other.bottom_left.x
            or self.bottom_left.x > other.top_right.x
            or self.top_right.y < other.bottom_left.y
            or self.bottom_left.y > other.top_right.y
        )


class Point:
    """getting the points of rectangle"""

    def __init__(self, xcoord=0, ycoord=0):
        self.x = xcoord
        self.y = ycoord


class OakdDetection:
    """OAK-D Detection Class."""

    LOG = Logger("Oakd Detection")

    def __init__(self, camera_name: str, debug: bool = True):
        assert camera_name in OAKDConfigs.device_ids, "Invalid camera name"

        self._device_id = OAKDConfigs.device_ids[camera_name]
        self._camera_name = camera_name
        self._debug = debug
        self._lock = Lock()

        self._result = {
            "pose": {"x": 0, "y": 0, "z": 0},
            "threat_angle": 0,
            "is_detected": False,
            "is_near_threat": False,
        }

        device_info = dai.Device.getAllAvailableDevices()
        for device in device_info:
            if device.getMxId() == self._device_id:
                self._device_info = device
                break

        self._apply_safety_perimeter = ThreatDetectionParameters.apply_safety_perimeter
        if self._apply_safety_perimeter:
            self._safety_perimeter_radius = (
                ThreatDetectionParameters.safety_perimeter_radius
            )

        if self._debug:
            self._visualize_tracking = ThreatDetectionParameters.visualize_tracking
            self._visualize_weapon = ThreatDetectionParameters.visualize_weapon

        self.model_path = os.path.join(ROOT_DIR, "models", "yolov8n_rgb_12shave.blob")
        self.classes = CustomDataset.labels
        self.nnPath = self.model_path
        self.syncNN = True
        self._stop_thread = False

    def create_pipeline(self):
        self.pipeline = dai.Pipeline()
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        yolo_spatial_network = self.pipeline.create(
            dai.node.YoloSpatialDetectionNetwork
        )
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        nnNetworkOut = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xoutNN = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xout_depth.setStreamName("depth")
        nnNetworkOut.setStreamName("nnNetwork")
        camRgb.setPreviewSize(640, 480)
        camRgb.setPreviewKeepAspectRatio(False)
        camRgb.setIspScale(2, 3)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        # camRgb.initialControl.setManualFocus(100) # 0..255
        # camRgb.initialControl.setSharpness(4)     # range: 0..4, default: 1
        # camRgb.initialControl.setLumaDenoise(4)   # range: 0..4, default: 1
        # camRgb.initialControl.setChromaDenoise(4) # range: 0..4, default: 1
        # camRgb.setFps(40)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(
            mono_left.getResolutionWidth(), mono_left.getResolutionHeight()
        )
        yolo_spatial_network.setBlobPath(self.nnPath)
        yolo_spatial_network.setConfidenceThreshold(0.1)
        yolo_spatial_network.input.setBlocking(False)
        yolo_spatial_network.setBoundingBoxScaleFactor(0.5)
        yolo_spatial_network.setDepthLowerThreshold(100)
        yolo_spatial_network.setDepthUpperThreshold(5000)
        yolo_spatial_network.setNumInferenceThreads(2)
        yolo_spatial_network.setNumNCEPerInferenceThread(1)
        yolo_spatial_network.input.setQueueSize(1)
        yolo_spatial_network.setNumClasses(7)
        yolo_spatial_network.setCoordinateSize(4)
        yolo_spatial_network.setAnchors([])
        yolo_spatial_network.setAnchorMasks({})
        yolo_spatial_network.setIouThreshold(0.5)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        camRgb.preview.link(yolo_spatial_network.input)
        if self.syncNN:
            yolo_spatial_network.passthrough.link(xout_rgb.input)
        else:
            camRgb.preview.link(xout_rgb.input)

        yolo_spatial_network.out.link(xoutNN.input)
        stereo.depth.link(yolo_spatial_network.inputDepth)
        yolo_spatial_network.passthroughDepth.link(xout_depth.input)
        yolo_spatial_network.outNetwork.link(nnNetworkOut.input)
        return self.pipeline

    def center(self, bbox):
        return ((bbox[2] + bbox[0]) / 2, (bbox[3] + bbox[1]) / 2)

    def getDistance(self, xmin, ymin, xmax, ymax):
        """Distance between the two points"""

        return math.sqrt((xmin - xmax) ** 2 + (ymin - ymax) ** 2)

    def format_result(
        self,
        name,
        frame,
        detections,
        depthFrame,
    ):
        depth_downscaled = depthFrame[::4]
        if np.any(depth_downscaled != 0):
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
            max_depth = np.percentile(depth_downscaled, 99)
        else:
            min_depth = 0
            max_depth = 1
        depthFrameColor = np.interp(
            depthFrame, (min_depth, max_depth), (0, 255)
        ).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
        height = frame.shape[0]
        width = frame.shape[1]
        person_coords = []
        person_spatial = []
        pistol_coords = []

        rifle_coords = []

        for detection in detections:
            roiData = detection.boundingBoxMapping
            roi = roiData.roi
            roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])

            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)

            bbox = np.array([x1, y1, x2, y2])
            try:
                label = self.labelMap[detection.label]
            except:
                label = detection.label

            # print("labels",label)
            if label == 3:
                centers_person = []
                person_coords.append(bbox.tolist())
                person_spatial.append(
                    (
                        int(detection.spatialCoordinates.x),
                        int(detection.spatialCoordinates.y),
                        int(detection.spatialCoordinates.z),
                    )
                )

                for box in person_coords:
                    center_p = self.center(box)
                    centers_person.append(center_p)

            elif label == 1:
                rifle_coords.append(bbox.tolist())
                centers_rifle = [
                    (rifle_coords[0][0] + rifle_coords[0][2]) / 2,
                    (rifle_coords[0][1] + rifle_coords[0][3]) / 2,
                ]

            elif label == 0:
                pistol_coords.append(bbox.tolist())

            if len(rifle_coords) > 0 and len(person_coords) > 0:
                dis_person_gun = []
                real_cords_person = []
                for ele in centers_person:
                    dis_person_gun.append(
                        self.getDistance(
                            ele[0],
                            ele[1],
                            centers_rifle[0],
                            centers_rifle[1],
                        )
                    )
                min_index = dis_person_gun.index(min(dis_person_gun))
                target_coords = person_coords[min_index]
                real_cords_person = person_spatial[min_index]
                l1 = target_coords[0], target_coords[1]
                r1 = target_coords[2], target_coords[3]
                l2 = rifle_coords[0][0], rifle_coords[0][1]
                r2 = rifle_coords[0][2], rifle_coords[0][3]
                r3 = Rectangle(Point(l1), Point(r1), "blue")
                r4 = Rectangle(Point(l2), Point(r2), "red")
                if r3.intersects(r4):
                    # print("target detected")
                    frame = cv2.rectangle(
                        frame,
                        (target_coords[0], target_coords[1]),
                        (target_coords[2], target_coords[3]),
                        (0, 0, 255),
                        15,
                    )
                    x = real_cords_person[0] * 0.001
                    z = real_cords_person[2] * 0.001
                    y = real_cords_person[1] * 0.001
                    # if self._debug:
                    #     self._debug_result("rgb", frame, roi, depthData)

                    return (x, y, z), True

        return (0, 0, 0), False

    def _debug_result(self, name, frame, roi, depthData):
        """Debugging function to visualize the detections."""
        height, width = frame.shape[:2]
        color = [255, 255, 0]
        xmin, ymin = int(roi.topLeft().x), int(roi.topLeft().y)
        xmax, ymax = int(roi.bottomRight().x), int(roi.bottomRight().y)
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 1)
        cv2.putText(
            frame,
            f"X: {int(depthData.spatialCoordinates.x)} mm",
            (xmin + 10, ymin + 20),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            color,
        )
        cv2.putText(
            frame,
            f"Y: {int(depthData.spatialCoordinates.y)} mm",
            (xmin + 10, ymin + 35),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            color,
        )
        cv2.putText(
            frame,
            f"Z: {int(depthData.spatialCoordinates.z)} mm",
            (xmin + 10, ymin + 50),
            cv2.FONT_HERSHEY_TRIPLEX,
            0.5,
            color,
        )
        # cv2.imshow(name,frame)

    def update(self):
        with dai.Device(
            self.pipeline, self._device_info, maxUsbSpeed=dai.UsbSpeed.SUPER
        ) as device:
            previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            detectionNNQueue = device.getOutputQueue(
                name="detections", maxSize=4, blocking=False
            )
            depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
            networkQueue = device.getOutputQueue(
                name="nnNetwork", maxSize=4, blocking=False
            )

            while True:
                inPreview = previewQueue.get()
                inDet = detectionNNQueue.get()
                depth = depthQueue.get()
                inNN = networkQueue.get()

                frame = inPreview.getCvFrame()
                depthFrame = depth.getFrame()
                detections = inDet.detections
                depth, is_detected = self.format_result(
                    "rgb", frame, detections, depthFrame
                )
                # In world frame, x is forward, y is left, z is up
                # In camera frame, x is right, y is down, z is forward
                self._result["pose"] = {
                    "x": depth[2],
                    "y": -depth[0],
                    "z": -depth[1],
                }
                self._result["threat_angle"] = np.arctan2(depth[0], depth[1])
                self._result["is_detected"] = is_detected
                self._result["is_near_threat"] = False

                if self._debug:
                    cv2.imshow(f"Oakd Detections: {self._camera_name}", frame)
                    cv2.imshow(f"Oakd Depth: {self._camera_name}", depthFrame)
                    cv2.waitKey(1)

    @property
    def result(self):
        with self._lock:
            return self._result

    @property
    def stop_thread(self):
        with self._lock:
            return self._stop_thread

    def stop(self):
        self._stop_thread = True


def main():
    l = OakdDetection(camera_name="front_oakd_camera", debug=True)

    from threading import Thread

    try:
        thread_1 = Thread(target=l.update, daemon=True)
        thread_1.start()

        while True:
            # l.result
            print(l.result)
    except KeyboardInterrupt:
        l.stop()
        thread_1.join()
