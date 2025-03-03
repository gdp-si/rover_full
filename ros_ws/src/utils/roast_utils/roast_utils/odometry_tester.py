"""Odometry calibration testing for the robot to determine the error in the odometry."""
import os
import time
from copy import deepcopy

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


class OdometryCalibrator(Node):
    """Odometry Calibration & Tester definition."""

    _behavior_tree = os.path.join(
        get_package_share_directory("roast_utils"),
        "behavior_trees",
        "navigate_without_recovery.xml",
    )

    def __init__(self):
        super().__init__("odometry_calibrator", allow_undeclared_parameters=True)

        # Declare parameters
        self._side_length = self.declare_parameter("side_length", 1.0).value
        self._odom_topic = self.get_parameter_or("odom_topic", "odometry")

        # Subscribe to odometry
        self._odom_sub = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 10
        )

        self._odom_msg = Odometry()

        # Setup actions and services
        self._nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses"
        )

        # Wait for services and actions to be available
        self.get_logger().info("Waiting for actions to be available...")
        self._nav_through_poses_client.wait_for_server()
        self.get_logger().info("All services and actions are available.")

    def destroy_node(self):
        """Destroy the node."""
        self._nav_through_poses_client.destroy()
        self.destroy_node()

    def _odom_callback(self, msg):
        """
        Get odometer data.

        Args:
        ----
            msg (Odometry): Odometry message

        """
        self._odom_msg = msg

    def _get_poses(self, side_length):
        """
        Get the poses for the square path.

        Args:
        ----
            side_length (float): Length of the side of the square

        """
        _poses = []
        current_pose_x = self._odom_msg.pose.pose.position.x
        current_pose_y = self._odom_msg.pose.pose.position.y

        pose = PoseStamped()
        pose.header.frame_id = "map"

        # Move in the x direction
        pose.pose.position.x = current_pose_x + side_length
        pose.pose.position.y = current_pose_y
        _poses.append(deepcopy(pose))

        # Move in the y direction
        pose.pose.position.x = current_pose_x + side_length
        pose.pose.position.y = current_pose_y + side_length
        _poses.append(deepcopy(pose))

        # Move in the x direction
        pose.pose.position.x = current_pose_x
        pose.pose.position.y = current_pose_y + side_length
        _poses.append(deepcopy(pose))

        # Move in the y direction
        pose.pose.position.x = (
            current_pose_x - 0.0001
        )  # To avoid the robot getting stuck
        pose.pose.position.y = current_pose_y
        _poses.append(deepcopy(pose))

        return _poses

    def run(self):
        """Start the calibration tester."""
        while rclpy.ok():
            rclpy.spin_once(self)

            # Compute poses
            self._poses = self._get_poses(self._side_length)

            self._navigate_through_poses(self._poses)

            while self.goal_handle is not None and not self.result_future.done():
                rclpy.spin_once(self)

            pose = self._poses[-1]
            self.get_logger().info(
                f"Robot should be at x: {pose.pose.position.x} y: {pose.pose.position.y}"
            )
            # Print the error in odometry
            self._get_error_data(pose, self._feedback)

            # Wait for 3 seconds before moving to the next round
            time.sleep(3)

    def _feedback_callback(self, msg):
        self._feedback = msg.feedback

    def _navigate_through_poses(self, poses):
        while not self._nav_through_poses_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "'NavigateThroughPoses' action server not available, waiting..."
            )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses
        goal_msg.behavior_tree = self._behavior_tree

        self.get_logger().info(f"Navigating with {len(goal_msg.poses)} goals....")
        send_goal_future = self._nav_through_poses_client.send_goal_async(
            goal_msg, self._feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error(f"Goal with {len(poses)} poses was rejected!")
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def _get_error_data(self, pose, feedback):
        """Get the error in odometry."""
        error_x = round(pose.pose.position.x - feedback.current_pose.pose.position.x, 2)
        error_y = round(pose.pose.position.y - feedback.current_pose.pose.position.y, 2)
        error_x_percent = round(
            (
                (error_x / pose.pose.position.x) * 100
                if pose.pose.position.x != 0
                else 0
            ),
            2,
        )
        error_y_percent = round(
            (
                (error_y / pose.pose.position.y) * 100
                if pose.pose.position.y != 0
                else 0
            ),
            2,
        )

        self.get_logger().info(f"Error in x: {error_x} y: {error_y}")
        self.get_logger().info(f"Error in x: {error_x_percent}% y: {error_y_percent}%")
        print("=====" * 10)


def main():
    """Launch the main function."""
    rclpy.init()
    odometry_calibrator = OdometryCalibrator()
    odometry_calibrator.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
