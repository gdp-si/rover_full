#!usr/bin/python3
"""Create the ROS2 robot state publisher for the roast robot."""
import os

import rclpy
from geometry_msgs.msg import Pose, TransformStamped
from rclpy.lifecycle import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from roast.glogging import Logger
from roast.robot import RobotParameters
from roast.utils import euler_to_quaternion


class RobotStatePublisher(Node):
    """The robot state publisher uses tf2 packages for ROS2. In
    other works, tf2 broadcaster for the robot"""

    LOG = Logger(_module_name=os.path.basename(__file__))

    SENSOR_POSITIONS = RobotParameters.SENSOR_POSITIONS
    WHEEL_POSITIONS = RobotParameters.WHEEL_POSITIONS

    header_frame_id = "base_link"

    def __init__(self):
        super().__init__("robot_state_publisher")

        self.namespace = self.get_namespace()

        # Initialize the transform broadcaster
        self._broadcaster = {}
        self.base_foot_print_static = StaticTransformBroadcaster(self)
        for sensor in self.SENSOR_POSITIONS:
            self._broadcaster[sensor] = StaticTransformBroadcaster(self)

        # Setup messages
        self.robot_pose = Pose()

        # Publish static transforms
        self.static_broadcaster()

    def destroy_node(self):
        """Destroy the robot state publisher node"""
        self.LOG.INFO("Destroying robot state publisher node")
        super().destroy_node()

    def static_broadcaster(self):
        """Broadcast positions of sensors and wheels"""
        # Broadcast sensor positions
        for sensor in self.SENSOR_POSITIONS:
            self.broadcast_sensor_position(sensor)

        # Broadcast base_foot_print position
        self.broadcast_base_foot_print()

    def broadcast_base_foot_print(self):
        """Broadcast base_foot_print position"""

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = self.header_frame_id

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = RobotParameters.ROBOT_BASE_FRAME["z"]

        self.base_foot_print_static.sendTransform(t)

    def broadcast_sensor_position(self, sensor: str):
        """Broadcast sensor position

        Args:
            sensor (str): Sensor name
        """
        self.LOG.INFO(f"Starting Transformation for {sensor} -> {self.header_frame_id}")
        sensor_pose = self.SENSOR_POSITIONS[sensor]
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.header_frame_id
        t.child_frame_id = sensor

        t.transform.translation.x = sensor_pose["x"]
        t.transform.translation.y = sensor_pose["y"]
        t.transform.translation.z = sensor_pose["z"]

        q_angle = euler_to_quaternion(
            (sensor_pose["roll"], sensor_pose["pitch"], sensor_pose["yaw"])
        )
        t.transform.rotation.x = q_angle[0]
        t.transform.rotation.y = q_angle[1]
        t.transform.rotation.z = q_angle[2]
        t.transform.rotation.w = q_angle[3]

        self._broadcaster[sensor].sendTransform(t)

    def broadcast_wheel_position(self, wheel: str):
        """Broadcast wheel position

        Args:
            wheel (str): Wheel name
        """
        wheel_pose = self.WHEEL_POSITIONS[wheel]
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        if self.namespace == "":
            t.header.frame_id = self.header_frame_id
            t.child_frame_id = wheel
        else:
            t.header.frame_id = f"{self.namespace}/{self.header_frame_id}"
            t.header.child_frame_id = f"{self.namespace}/{wheel}"

        t.transform.translation.x = wheel_pose[0]
        t.transform.translation.y = wheel_pose[1]
        t.transform.translation.z = wheel_pose[2]

        q_angle = euler_to_quaternion((wheel_pose[3], wheel_pose[4], wheel_pose[5]))
        t.transform.rotation.x = q_angle[0]
        t.transform.rotation.y = q_angle[1]
        t.transform.rotation.z = q_angle[2]
        t.transform.rotation.w = q_angle[3]

        # To transform joint states to tf2
        # self.br.sendTransform(t)


def main():
    """Main function"""
    rclpy.init(args=None)

    try:
        robot_state_publisher = RobotStatePublisher()
        rclpy.spin(robot_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_state_publisher.destroy_node()


if __name__ == "__main__":
    main()
