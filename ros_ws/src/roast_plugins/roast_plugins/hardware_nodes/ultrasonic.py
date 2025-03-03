"""ROS Wrapper for the Range Sensor Plugin."""
import os
from math import inf

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from roast_plugins import ROAST_PROFILE
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

from roast.glogging import Logger
from roast.io import Ultrasonic as ULTRASONIC
from roast.io.configs import UltraConfig
from roast.robot import RobotParameters
from roast.utils import euler_to_quaternion


class UltrasonicPublisher(Node):
    """Robot Range Sensor Publisher

    Args:
        Node (rclpy.Node): Node Object
    """

    MIN_THRESHOLD = RobotParameters.threshold_range

    LOG = Logger(_module_name=os.path.basename(__file__))

    def __init__(self, ultrasonic_sensor: Range, mode: str = "VARIABLE"):
        assert mode in ["VARIABLE", "FIXED"], "Mode must be VARIABLE or FIXED"
        self._mode = mode

        self.ULTRASONIC = ultrasonic_sensor
        ultra_side = str(self.ULTRASONIC)
        super().__init__(f"{ultra_side}_ultrasonic_publisher")
        self._range_broadcaster = TransformBroadcaster(self)
        self.sensor_position = RobotParameters.SENSOR_POSITIONS[f"{ultra_side}"]

        self.front_publisher = self.create_publisher(
            Range, f"{ultra_side}_ultrasonic/data", ROAST_PROFILE
        )
        self._status_publisher = self.create_publisher(
            Bool, "hardware_info/status/primary/range", qos_profile=ROAST_PROFILE
        )

        # SET RANGE
        self._MIN_RANGE = UltraConfig.min_range
        self._MAX_RANGE = UltraConfig.max_range

        self._range_msg = Range()
        if self.get_namespace() == "/":
            self._range_msg.header.frame_id = f"{ultra_side}_range"
        else:
            self._range_msg.header.frame_id = (
                f"{self.get_namespace()}/{ultra_side}_range"
            )
        if UltraConfig.radiation_type != "infrared":
            self._range_msg.radiation_type = Range.ULTRASOUND
        else:
            self._range_msg.radiation_type = Range.INFRARED
        self._range_msg.field_of_view = UltraConfig.field_of_view
        if self._mode == "VARIABLE":
            self._range_msg.min_range = self._MIN_RANGE
            self._range_msg.max_range = self._MAX_RANGE
        else:
            self._range_msg.min_range = UltraConfig.fixed_range
            self._range_msg.max_range = UltraConfig.fixed_range

        # Check if sensors are active
        self.ULTRASONIC.is_active()

        # Setup Timer
        timer_period = 1 / RobotParameters.PUBLISH_RATE["ultrasonic"]  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        status_timer_period = 1 / RobotParameters.PUBLISH_RATE["status"]  # seconds
        self.timer = self.create_timer(status_timer_period, self.status_timer_callback)

    def destroy_node(self):
        """Destroy node"""
        self.LOG.INFO("Shutting down Range Sensor publisher...")
        self.destroy_node()
        rclpy.shutdown()

    def timer_callback(self):
        """Callback function for timer"""
        dist = self.ULTRASONIC.update()
        if self._mode == "VARIABLE":
            self._publish_variable_data(dist)
        else:
            self._publish_fixed_data(dist)
        self.transform_broadcast()

    def status_timer_callback(self):
        self._status_publisher.publish(Bool(data=self.ULTRASONIC.is_active()))

    def _publish_variable_data(self, msg):
        """Publish Range data to ros topic

        Args:
            msg (dict): Range data to be published
        """
        self._range_msg.header.stamp = self.get_clock().now().to_msg()
        self._range_msg.range = float(msg)
        self.front_publisher.publish(self._range_msg)

    def _publish_fixed_data(self, msg):
        """Publish Range data to ros topic

        Args:
            msg (dict): Range data to be published
        """
        self._range_msg.header.stamp = self.get_clock().now().to_msg()
        if float(msg) <= self._range_msg.max_range:
            self._range_msg.range = -inf
        else:
            self._range_msg.range = inf
        self.front_publisher.publish(self._range_msg)

    def transform_broadcast(self):
        """Broadcast transform to tf tree

        Args:
            msg (dict): Range data to be published
        """

        t = TransformStamped()
        t.header.stamp = self._range_msg.header.stamp
        t.header.frame_id = "base_link"
        t.child_frame_id = self._range_msg.header.frame_id

        t.transform.translation.x = self.sensor_position["x"]
        t.transform.translation.y = self.sensor_position["y"]
        t.transform.translation.z = self.sensor_position["z"]

        q = euler_to_quaternion(
            (
                self.sensor_position["roll"],
                self.sensor_position["pitch"],
                self.sensor_position["yaw"],
            )
        )

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self._range_broadcaster.sendTransform(t)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    executor.add_node(UltrasonicPublisher(ULTRASONIC("front"), mode=UltraConfig.mode))
    executor.add_node(UltrasonicPublisher(ULTRASONIC("left"), mode=UltraConfig.mode))
    executor.add_node(UltrasonicPublisher(ULTRASONIC("right"), mode=UltraConfig.mode))

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()


if __name__ == "__main__":
    main()
