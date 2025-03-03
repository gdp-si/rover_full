"""
Stream the coordinates of the threat from a CSV file to the threat_pose topic.

- CSV file is of format: is_threat, y, x, threat_type, y_safe, x_safe
- ROS CSV file is of format: timestamp, timestamp_ns, frame, x, y, z, qx, qy, qz, qw

Usage:
ros2 run roast_ai threat_pose_publisher_csv -p type:=ros -p file_path:=<path_to_csv_file>
ros2 run roast_ai threat_pose_publisher_csv -p type:=py -p file_path:=<path_to_csv_file>
"""
import csv

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from roast_plugins import ROAST_PROFILE
from std_msgs.msg import Bool

from roast.ai.configs import TargetTrackingConfigs


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


class ThreatPosePublisher(Node):
    """Publish the threat pose from a CSV file."""

    def __init__(self):
        """Initialize the node and the publisher."""
        super().__init__("threat_pose_publisher_csv")
        self.publisher_ = self.create_publisher(
            PoseStamped,
            "threat_pose/safe_pose",
            qos_profile=ROAST_PROFILE,
        )

        self.is_threat_found_pub = self.create_publisher(
            Bool,
            "threat_found",
            qos_profile=ROAST_PROFILE,
        )

        # Declare and acquire parameters
        try:
            self.type = self.declare_parameter("type", "").value
            self.file_path = self.declare_parameter("file_path", "").value
        except Exception as e:
            raise Exception(f"Failed to declare parameters: {e}") from e

        timer_period = 1 / TargetTrackingConfigs.update_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.threat_pose = PoseStamped()
        self.threat_pose.header.frame_id = "camera"
        self.threat_pose.pose.orientation.w = 1.0

        with open(self.file_path, "r", encoding="utf-8") as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=",")
            self.csv_data = list(csv_reader)

    def timer_callback(self):
        """Publish the threat pose from a CSV file."""
        row = self.csv_data.pop(0)
        self.threat_pose.header.stamp = self.get_clock().now().to_msg()
        if self.type == "py":
            data = True if int(row[0]) == 1 else False
            self.is_threat_found_pub.publish(Bool(data=data))
            try:
                self.threat_pose.pose.position.y = float(row[4])
                self.threat_pose.pose.position.x = float(row[5])
            except ValueError:
                self.threat_pose.pose.position.x = 0.0
                self.threat_pose.pose.position.y = 0.0
                self.threat_pose.pose.position.z = 0.0
            try:
                q = quaternion_from_euler(0, 0, float(row[6]))
                self.threat_pose.pose.orientation.x = q[0]
                self.threat_pose.pose.orientation.y = q[1]
                self.threat_pose.pose.orientation.z = q[2]
                self.threat_pose.pose.orientation.w = q[3]
            except ValueError:
                self.threat_pose.pose.orientation.x = 0.0
                self.threat_pose.pose.orientation.y = 0.0
                self.threat_pose.pose.orientation.z = 0.0
                self.threat_pose.pose.orientation.w = 1.0
        elif self.type == "ros":
            self.is_threat_found_pub.publish(Bool(data=True))
            self.threat_pose.pose.position.x = float(row[3])
            self.threat_pose.pose.position.y = float(row[4])
            self.threat_pose.pose.orientation.x = float(row[6])
            self.threat_pose.pose.orientation.y = float(row[7])
            self.threat_pose.pose.orientation.z = float(row[8])
            self.threat_pose.pose.orientation.w = float(row[9])

        else:
            print("Invalid type. Please choose 'py' or 'ros'.")
            return

        self.publisher_.publish(self.threat_pose)


def main(args=None):
    """Run the node."""
    rclpy.init(args=args)
    threat_pose_publisher = ThreatPosePublisher()
    rclpy.spin(threat_pose_publisher)
    threat_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
