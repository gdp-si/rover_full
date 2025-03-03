"""ROS Wrappers for the Roast IMU."""
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from roast_plugins import ROAST_PROFILE
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from roast.glogging import Logger
from roast.io import Imu as IMU
from roast.robot import RobotParameters


class IMUPublisher(Node):
    """Imu publisher for roast robot

    Args:
        Node (rclpy.Node): Node Object
    """

    LOG = Logger(_module_name="IMU Publisher")

    def __init__(self):
        super().__init__("imu_publisher")
        self._publisher: Optional[Publisher] = None
        self._status_publisher: Optional[Publisher] = None
        self._topic_timer: Optional[Timer] = None
        self._status_timer: Optional[Timer] = None
        self._msg = Imu()
        self._status_msg = Bool()
        self._reset_service = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Configuring IMU publisher")
        self._imu_module = IMU()
        time.sleep(3)

        # Setup messages
        if self.get_namespace() != "/":
            self._msg.header.frame_id = f"/{self.get_namespace()}/imu"
        else:
            self._msg.header.frame_id = "imu_link"
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.orientation = Quaternion()
        self._msg.orientation_covariance = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]  # [1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6]
        self._msg.angular_velocity_covariance[0] = 0  # -1
        self._msg.linear_acceleration_covariance[0] = 0  # -1

        # Check if IMU is active
        if not self._imu_module.is_active():
            self.LOG.ERROR("IMU is not active")
            return TransitionCallbackReturn.FAILURE

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._publisher = self.create_lifecycle_publisher(
            Imu, "imu", qos_profile=ROAST_PROFILE
        )

        self._status_publisher = self.create_lifecycle_publisher(
            Bool, "hardware_info/status/primary/imu", qos_profile=ROAST_PROFILE
        )

        self._reset_service = self.create_service(
            SetBool,
            f"{self.get_name()}/calibrate_imu",
            self._calibrate_imu_callback,
            qos_profile=ROAST_PROFILE,
        )
        # Setup Timer
        timer_period = 1 / RobotParameters.PUBLISH_RATE["imu"]  # seconds
        self._topic_timer = self.create_timer(timer_period, self.timer_callback)

        status_timer_period = 1 / RobotParameters.PUBLISH_RATE["status"]  # seconds
        self._status_timer = self.create_timer(
            status_timer_period, self.status_timer_callback
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Deactivating IMU publisher")

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Cleaning up IMU publisher")

        self.destroy_publisher(self._publisher)
        self.destroy_publisher(self._status_publisher)
        self.destroy_timer(self._topic_timer)
        self.destroy_timer(self._status_timer)
        self.destroy_service(self._reset_service)
        self._imu_module = None

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Shutting down IMU publisher")
        self.destroy_publisher(self._publisher)
        self.destroy_publisher(self._status_publisher)
        self.destroy_timer(self._topic_timer)
        self.destroy_timer(self._status_timer)
        self.destroy_service(self._reset_service)
        self._imu_module = None

        return super().on_shutdown(state)

    def publish(self, msg):
        """Publish imu data to ros topic

        Args:
            msg (dict): IMU data to be published
        """
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.orientation = Quaternion()
        self._msg.orientation.x = msg["quaternion"][0]
        self._msg.orientation.y = msg["quaternion"][1]
        self._msg.orientation.z = msg["quaternion"][2]
        self._msg.orientation.w = msg["quaternion"][3]

        self._msg.linear_acceleration.x = msg["linear_acceleration"][0]
        self._msg.linear_acceleration.y = msg["linear_acceleration"][1]
        self._msg.linear_acceleration.z = msg["linear_acceleration"][2]

        self._publisher.publish(self._msg)

    def _calibrate_imu_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        if request.data:
            response.success = self._imu_module.calibrate_imu()

        response.success = True

        return response

    def destroy_node(self):
        """Destroy node"""
        self.LOG.INFO("Shutting down IMU publisher...")
        self.destroy_publisher(self._publisher)
        self.destroy_publisher(self._status_publisher)
        self.destroy_timer(self._topic_timer)
        self.destroy_timer(self._status_timer)
        self.destroy_service(self._reset_service)
        self._imu_module.destroy()
        super().destroy_node()

    def timer_callback(self):
        """Callback function for timer"""
        if (
            self._publisher is None
            or not self._publisher.is_activated
            or not self._imu_module.is_active()
        ):
            return

        imu_data = self._imu_module.update()
        self.publish(imu_data)

    def status_timer_callback(self):
        """Callback function for timer"""
        if self._status_publisher is None or not self._status_publisher.is_activated:
            return

        self._status_publisher.publish(Bool(data=self._imu_module.is_active()))


def main():
    """Main function"""
    rclpy.init(args=None)

    try:
        imu_publisher = IMUPublisher()
        rclpy.spin(imu_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.destroy_node()


if __name__ == "__main__":
    main()
