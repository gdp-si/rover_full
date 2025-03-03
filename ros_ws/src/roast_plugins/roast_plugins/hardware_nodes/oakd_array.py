import threading
from typing import Dict

import depthai as dai
import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import LifecycleNode, Publisher, State, TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState
from sensor_msgs.msg import Image

from roast import RobotProfile
from roast.glogging import Logger
from roast.io import OAKDConfigs


class MultiCameraPublisher(LifecycleNode):
    """Oakd camera publisher for roast robot"""

    LOG = Logger(_module_name="oakd_array_publisher")

    def __init__(self, rgb_camera: bool = False):
        super().__init__("oakd_array_publisher")
        self._mono_left_pub: Dict[str, Publisher] = {}
        self._color_camera_pub: Dict[str, Publisher] = {}
        self._threads: Dict[str, threading.Thread] = {}
        self._device_infos = dai.Device.getAllAvailableDevices()
        self._device_mxids = [
            device_info.getMxId() for device_info in self._device_infos
        ]

        self._stop_node = False
        self._target_mxids = {}
        self._rgb_camera = True
        for camera in RobotProfile.OAKD_ARRAY_LIST:
            if camera in OAKDConfigs.device_ids:
                self._target_mxids[camera] = OAKDConfigs.device_ids[camera]

        self.pipeline = dai.Pipeline()

    def _start_camera(self, target_device: str):
        target_device_id = self._target_mxids[target_device]
        self._mono_left_pub[target_device] = self.create_lifecycle_publisher(
            Image, f"{target_device}/mono_left", 10
        )

        if self._rgb_camera:
            self._color_camera_pub[target_device] = self.create_lifecycle_publisher(
                Image, f"{target_device}/rgb", 10
            )

        thread = threading.Thread(
            target=self._start_publish_thread,
            args=(target_device, target_device_id),
            daemon=True,
        )
        thread.start()
        self._threads[target_device] = thread

    def _start_publish_thread(self, target_device: str, target_device_id: str):
        mono_left_publisher = self._mono_left_pub[target_device]
        device = dai.Device(target_device_id)
        device.startPipeline(self.pipeline)
        mono_left_queue = device.getOutputQueue(
            name="mono_left", maxSize=4, blocking=False
        )
        bridge = CvBridge()
        mono_left_msg = Image()
        publish_rgb = self._rgb_camera

        mono_left_msg.header.frame_id = target_device
        mono_left_msg.header.frame_id = f"{str(target_device)}"  # target_device
        mono_left_msg.height = 400
        mono_left_msg.width = 640
        mono_left_msg.step = mono_left_msg.width
        mono_left_msg.encoding = "mono8"

        rgb_image = Image()
        color_queue = None
        if publish_rgb:
            color_queue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            rgb_image.header.frame_id = target_device
            rgb_image.header.frame_id = f"{str(target_device)}"  # target_device
            rgb_image.height = 1080
            rgb_image.width = 1920
            rgb_image.step = rgb_image.width * 3
            rgb_image.encoding = "bgr8"

        while rclpy.ok() and not self._stop_node:
            if not mono_left_publisher.is_activated:
                self.LOG.DEBUG("Camera publisher not activated")
                continue

            mono_left_frame = mono_left_queue.get().getCvFrame()
            mono_left_msg.data = bridge.cv2_to_imgmsg(
                mono_left_frame, encoding="mono8"
            ).data

            mono_left_msg.header.stamp = self.get_clock().now().to_msg()
            mono_left_publisher.publish(mono_left_msg)

            if publish_rgb:
                rgb_frame = color_queue.get().getCvFrame()
                rgb_image.data = bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8").data
                rgb_image.header.stamp = self.get_clock().now().to_msg()
                self._color_camera_pub[target_device].publish(rgb_image)

        self.LOG.INFO("Stopping Oakdetect publisher")
        device.close()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        mono_left = self.pipeline.create(dai.node.MonoCamera)

        if self._rgb_camera:
            camera = self.pipeline.create(dai.node.ColorCamera)
            camera.setPreviewSize(1920, 1080)
            camera.setInterleaved(False)
            camera.setFps(10)
            camera.setBoardSocket(dai.CameraBoardSocket.RGB)
            camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)

            xout_rgb = self.pipeline.create(dai.node.XLinkOut)
            xout_rgb.setStreamName("rgb")
            camera.preview.link(xout_rgb.input)

        xout_mono_left = self.pipeline.create(dai.node.XLinkOut)

        xout_mono_left.setStreamName("mono_left")

        mono_left.out.link(xout_mono_left.input)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        for target_device in self._target_mxids:
            self._start_camera(target_device)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Deactivating Oakdetect publisher")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.LOG.INFO("Cleaning up Oakdetect publisher")
        for target_device in self._target_mxids:
            self.destroy_publisher(self._mono_left_pub[target_device])

        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.LOG.INFO("Shutting down Oakdetect publisher")
        for target_device in self._target_mxids:
            self.destroy_publisher(self._mono_left_pub[target_device])

        return super().on_shutdown(state)

    def destroy_node(self):
        """Destroy the threads and node"""
        self._stop_node = True
        for thread in self._threads.values():
            thread.join()

        for target_device in self._target_mxids:
            self.destroy_publisher(self._mono_left_pub[target_device])
        super().destroy_node()


def main():
    """Main function"""
    rclpy.init(args=None)
    multi_camera_publisher = MultiCameraPublisher(rgb_camera=True)
    try:
        rclpy.spin(multi_camera_publisher)
    except KeyboardInterrupt:
        multi_camera_publisher.destroy_node()


if __name__ == "__main__":
    main()
