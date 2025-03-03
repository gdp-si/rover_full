"""Camera defintion for accessing OAK-D Pro camera."""
from typing import Optional

import depthai as dai

from roast.glogging import Logger
from roast.utils.patterns import ThreadSafeSingletonMetaclass


class OAKDProConfig:
    # NOTE: The configurations should be balaced with the available resources

    PREVIEW_SIZE = (1920, 1080)
    PREVIEW_FPS = 30
    OPENVINO_VERSION = "2022.1"  # 2021.4, 2021.3, 2021.2, 2022.1

    RESOLUTION = "1080p"  # 1080p, 1200p, 12mp, 5mp, 4k

    USB_MODE = "SUPER"  # FULL, HIGH, LOW, SUPER, SUPER_PLUS

    # Queue sizes for the different streams
    POINTCLOUD_QUEUE_SIZE = 1
    DATA_QUEUE_SIZE = 1  # Depth, Disparity, Rectified, Colorized
    RGB_QUEUE_SIZE = 1

    # Mono Camera Parameters
    USE_RECTIFIED = False  # Highly computational expensive
    MONO_PREVIEW_SIZE = (400, 300)
    MONO_RESOLUTION = "400p"  # 720p, 800p, 400p, 480p

    # DEPTH Parameters
    KERNEL_SIZE = "3x3"  # 3x3, 5x5, 7x7, MEDIAN_OFF
    CONFIDENCE_THRESHOLD = 200  # 0-255
    STEREO_QUEUE_SIZE = 1
    OUTPUT_DEPTH = True


class OakDPro(metaclass=ThreadSafeSingletonMetaclass):
    LOG = Logger(_module_name="OAKDPro")

    def __init__(
        self,
        access_color: bool = True,
        access_depth: bool = True,
        access_mono: bool = True,
        access_spatial: bool = True,
        debug: bool = False,
        device_info: Optional[dai.DeviceInfo] = None,
    ):
        """Initialize OAK-D Pro camera.

        Args:
            access_color (bool): Access color. Defaults to True.
            access_depth (bool): Access depth. Defaults to True.
            access_mono (bool): Access mono cameras (left, right). Defaults to True.
            access_spatial (bool): Access spatial. Defaults to True.
            debug (bool): Debug mode. Defaults to False.
        """
        self._access_color = access_color
        self._access_depth = access_depth
        self._access_mono = access_mono
        self._access_spatial = access_spatial
        self._debug = debug  # TODO: Implement debug mode

        assert device_info is not None, "Device info must be provided"
        self._device_info = device_info

        # Get the configs for the camera
        self._configs = self._get_configs()

        (
            self._pipeline,
            self._usb_mode,
            self._openvino_version,
        ) = self._create_pipeline()

        # Create necessary pipelines
        if self._access_color:
            self._camera = self._pipeline.create(dai.node.ColorCamera)
            self._add_color_camera_pipeline()
        if self._access_mono or self._access_depth:
            # Get Unrectified Mono Cameras
            self._mono_left, self._mono_right = self._add_mono_camera_pipelines()
        if self._access_depth:
            # Get Rectified Mono Cameras and Depth+
            self._stereo = self._add_depth_pipeline(self._configs.USE_RECTIFIED)
        # if self._access_spatial:
        #     self._add_spatial_pipeline()

        # Create the device with updated pipeline
        self._device: Optional[dai.Device] = None

    def __del__(self):
        self._device.close()

    def create_device(self, pipeline: Optional[dai.Pipeline] = None):
        if pipeline:
            self._pipeline = pipeline

        self._device = dai.Device(self._pipeline, self._device_info, self._usb_mode)
        self._device.startPipeline()

        self.LOG.INFO("OAK-D Pro device created.")

    def _create_pipeline(self):
        """Create Pipeline for OAK-D Pro."""

        pipeline = dai.Pipeline()

        usb_modes = {
            "HIGH": dai.UsbSpeed.HIGH,
            "SUPER": dai.UsbSpeed.SUPER,
            "FULL": dai.UsbSpeed.FULL,
            "LOW": dai.UsbSpeed.LOW,
            "SUPER_PLUS": dai.UsbSpeed.SUPER_PLUS,
        }

        openvino_versions = {
            "2021.4": dai.OpenVINO.Version.VERSION_2021_4,
            "2021.3": dai.OpenVINO.Version.VERSION_2021_3,
            "2021.2": dai.OpenVINO.Version.VERSION_2021_2,
            "2022.1": dai.OpenVINO.Version.VERSION_2022_1,
        }

        return (
            pipeline,
            usb_modes[self._configs.USB_MODE],
            openvino_versions[self._configs.OPENVINO_VERSION],
        )

    def is_active(self):
        """Check if the OAK-D Pro is active."""
        with dai.Device(self._pipeline, self._usb_mode) as device:
            return device is not None

    @property
    def pipeline(self):
        return self._pipeline

    @property
    def camera(self):
        return self._camera

    @property
    def usb_mode(self):
        return self._usb_mode

    @property
    def stereo(self):
        return self._stereo

    @property
    def device(self):
        return self._device

    def _get_configs(self):
        """Get the configs for the camera."""
        return OAKDProConfig()

    def _add_color_camera_pipeline(self):
        """Add color camera pipeline to the OAK-D Pro pipeline."""
        self._camera.setPreviewSize(self._configs.PREVIEW_SIZE)
        self._camera.setInterleaved(False)

        RESOLUTIONS = {
            "1080p": dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            "4k": dai.ColorCameraProperties.SensorResolution.THE_4_K,
            "12mp": dai.ColorCameraProperties.SensorResolution.THE_12_MP,
            "5mp": dai.ColorCameraProperties.SensorResolution.THE_5_MP,
            "1200p": dai.ColorCameraProperties.SensorResolution.THE_1200_P,
        }

        self._camera.setResolution(RESOLUTIONS[self._configs.RESOLUTION])

        xlinkout = self._pipeline.create(dai.node.XLinkOut)
        xlinkout.setStreamName("rgb")
        xlinkout.input.setQueueSize(self._configs.DATA_QUEUE_SIZE)

        self._camera.preview.link(xlinkout.input)

        self.LOG.DEBUG("Color Camera Pipeline Added")

    def _add_mono_camera_pipelines(self):
        """Add mono camera pipelines to the OAK-D Pro pipeline."""
        cam_left = self._pipeline.create(dai.node.MonoCamera)
        cam_right = self._pipeline.create(dai.node.MonoCamera)
        xlinkout_left = self._pipeline.create(dai.node.XLinkOut)
        xlinkout_right = self._pipeline.create(dai.node.XLinkOut)

        cam_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        cam_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        RESOLUTIONS = {
            "720p": dai.MonoCameraProperties.SensorResolution.THE_720_P,
            "800p": dai.MonoCameraProperties.SensorResolution.THE_800_P,
            "400p": dai.MonoCameraProperties.SensorResolution.THE_400_P,
            "480p": dai.MonoCameraProperties.SensorResolution.THE_480_P,
        }

        cam_left.setResolution(RESOLUTIONS[self._configs.MONO_RESOLUTION])
        cam_right.setResolution(RESOLUTIONS[self._configs.MONO_RESOLUTION])

        xlinkout_left.setStreamName("stereo_left")
        xlinkout_right.setStreamName("stereo_right")

        cam_left.out.link(xlinkout_left.input)
        cam_right.out.link(xlinkout_right.input)

        self.LOG.DEBUG("Mono Camera Pipelines Added")

        return cam_left, cam_right

    def _add_depth_pipeline(self, rectified_mono: bool):
        depth = self._pipeline.create(dai.node.StereoDepth)
        xlinkout_depth = self._pipeline.create(dai.node.XLinkOut)

        KERNELS = {
            "3x3": dai.MedianFilter.KERNEL_3x3,
            "5x5": dai.MedianFilter.KERNEL_5x5,
            "7x7": dai.MedianFilter.KERNEL_7x7,
            "MEDIAN_OFF": dai.MedianFilter.MEDIAN_OFF,
        }
        depth.initialConfig.setMedianFilter(KERNELS[self._configs.KERNEL_SIZE])
        depth.setConfidenceThreshold(self._configs.CONFIDENCE_THRESHOLD)
        depth.setLeftRightCheck(True)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(False)
        depth.setOutputDepth(self._configs.OUTPUT_DEPTH)

        xlinkout_depth.setStreamName("stereo_disparity")
        xlinkout_depth.input.setQueueSize(self._configs.DATA_QUEUE_SIZE)

        self._mono_left.out.link(depth.left)
        self._mono_right.out.link(depth.right)
        depth.disparity.link(xlinkout_depth.input)

        self.LOG.DEBUG("Depth Pipeline Added")

        return depth

        if rectified_mono:
            xlinkout_left = self._pipeline.create(dai.node.XLinkOut)
            xlinkout_right = self._pipeline.create(dai.node.XLinkOut)
            xlinkout_left.setStreamName("stereo_left_rectified")
            xlinkout_right.setStreamName("stereo_right_rectified")
            xlinkout_left.input.setQueueSize(self._configs.DATA_QUEUE_SIZE)
            xlinkout_right.input.setQueueSize(self._configs.DATA_QUEUE_SIZE)
            depth.rectifiedLeft.link(xlinkout_left.input)
            depth.rectifiedRight.link(xlinkout_right.input)

            self.LOG.DEBUG("Rectified Mono Pipelines Added")

    def _add_spatial_pipeline(self):
        xlinkout = self._pipeline.create(dai.node.XLinkOut)
        xlinkout.setStreamName("spatial_data")
        xlinkout.input.setQueueSize(self._configs.DATA_QUEUE_SIZE)

        # TODO: Add spatial data to the pipeline
        self._camera.preview.link(xlinkout.input)

    def get_spatial_pipeline(self):
        """Get the cloud pipeline for the camera."""
        return self._device.getOutputQueue(
            name="spatial_data",
            maxSize=self._configs.POINTCLOUD_QUEUE_SIZE,
            blocking=False,
        )

    def get_color_cam_pipeline(self):
        """Get the image frame pipeline for the camera."""

        return self._device.getOutputQueue(
            name="rgb", maxSize=self._configs.POINTCLOUD_QUEUE_SIZE, blocking=False
        )

    def get_mono_cam_pipelines(self):
        """Get the mono camera pipelines for the camera."""
        if self._configs.USE_RECTIFIED and not self._access_mono:
            return (
                self._device.getOutputQueue(
                    name="stereo_left_rectified",
                    maxSize=self._configs.STEREO_QUEUE_SIZE,
                    blocking=False,
                ),
                self._device.getOutputQueue(
                    name="stereo_right_rectified",
                    maxSize=self._configs.STEREO_QUEUE_SIZE,
                    blocking=False,
                ),
            )

        return (
            self._device.getOutputQueue(
                name="stereo_left",
                maxSize=self._configs.STEREO_QUEUE_SIZE,
                blocking=False,
            ),
            self._device.getOutputQueue(
                name="stereo_right",
                maxSize=self._configs.STEREO_QUEUE_SIZE,
                blocking=False,
            ),
        )

    def get_depth_pipeline(self):
        """Get the depth pipeline for the camera."""
        return self._device.getOutputQueue(
            name="stereo_disparity",
            maxSize=self._configs.STEREO_QUEUE_SIZE,
            blocking=False,
        )
