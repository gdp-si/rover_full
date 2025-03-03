"""ROS Node for Tuning PID parameters for the skid steering robot."""
import os
from typing import Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.node import Node

from roast.glogging import Logger
from roast.robot.configs import RobotParameters

PKG_DIRECTORY = get_package_share_directory("roast_ai")


class PIDTuner(Node):
    """PID Tuning Node for Robot Control - Roast."""

    robot_pose = np.array([0.0, 0.0, 0.0])
    odom = Odometry()

    LOG = Logger(_module_name="ROS PID Tuner")

    behavior_tree = os.path.join(
        PKG_DIRECTORY, "behavior_trees/navigate_without_recovery.xml"
    )

    def __init__(self, mode: str = "straight") -> None:
        super().__init__("pid_tuner")

        if mode in ["straight", "square"]:
            self.mode = mode
        else:
            raise ValueError("Invalid mode. Accepted values are 'straight' or 'square'")

        # Setup PID as ROS parameters
        self.Kp = (
            self.declare_parameter("Kp", RobotParameters.Kp)
            .get_parameter_value()
            .double_value
        )
        self.Ki = (
            self.declare_parameter("Ki", RobotParameters.Ki)
            .get_parameter_value()
            .double_value
        )
        self.Kd = (
            self.declare_parameter("Kd", RobotParameters.Kd)
            .get_parameter_value()
            .double_value
        )

        # Setup ROS subscribers
        self._subscription = self.create_subscription(
            Odometry, "odometry", self.odom_callback, 10
        )

        self._error_publisher = self.create_publisher(PoseStamped, "error", 10)

        # Setup Navigator
        self.navigator = BasicNavigator()
        # set initial pose
        initial_pose = PoseStamped()
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)
        self.feedback = None
        # self.update_pid()
        self.LOG.INFO("PID Tuner Ready")

        side = float(input("Enter side length / distance based on the mode: "))
        self.goal_poses = self._get_poses(side)

    def _get_poses(self, side: float) -> list:
        """Get the goal poses in PoseStamped() for the robot to drive to."""
        goal_poses = []

        def ros_pose(x, y, theta):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = theta
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            return pose

        if self.mode == "straight":
            goal_poses.append(ros_pose(side, 0.0, 0.0))
        else:
            goal_poses.append(ros_pose(side, 0.0, 0.0))
            goal_poses.append(ros_pose(side, side, 0.0))
            goal_poses.append(ros_pose(0.0, side, 0.0))
            goal_poses.append(ros_pose(0.0, 0.0, 0.0))

        return goal_poses

    def odom_callback(self, msg):
        """
        Get odometry message.

        Args:
        ----
            msg (Odometry): Odometry message

        """
        self.odom = msg

    def get_robot_pose(self) -> Tuple[np.array, np.array]:
        """
        Get the current pose of the robot calculated from odometry.

        Returns
        -------
            pose (np.ndarray): Current pose of the robot

        """
        current_pose = np.array(
            [
                self.odom.pose.pose.position.x,
                self.odom.pose.pose.position.y,
                self.odom.pose.pose.orientation.z,
            ]
        )

        # TODO: Change to path
        self.feedback = self.navigator.getFeedback()

        expected_pose = np.array(
            [
                self.feedback.current_pose.pose.position.x,
                self.feedback.current_pose.pose.position.y,
                self.feedback.current_pose.pose.orientation.z,
            ]
        )

        self.LOG.DEBUG(f"Current Pose: {current_pose}; Expected Pose: {expected_pose}")

        return current_pose, expected_pose

    def update_pid(self):
        """Update the PID parameters."""
        self.Kp = (
            self.get_parameter("Kp", RobotParameters.Kp)
            .get_parameter_value()
            .double_value
        )
        self.Ki = (
            self.get_parameter("Ki", RobotParameters.Ki)
            .get_parameter_value()
            .double_value
        )
        self.Kd = (
            self.get_parameter("Kd", RobotParameters.Kd)
            .get_parameter_value()
            .double_value
        )

        self.LOG.DEBUG(f"Kp: {self.Kp}; Ki: {self.Ki}; Kd: {self.Kd}")

    def goal_response_callback(self, future):
        """Goal response callback."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        _get_result_future = goal_handle.get_result_async()
        _get_result_future.add_done_callback(self.get_result_callback)

    def start_tuning(self):
        """Start the PID tuning."""
        while rclpy.ok():
            # Send our route
            # self.update_pid()
            print(self.goal_poses)
            self.navigator.goThroughPoses(
                self.goal_poses, behavior_tree=self.behavior_tree
            )

            while not self.navigator.isTaskComplete():
                self.navigator.clearAllCostmaps()
                current_pose, expected_pose = self.get_robot_pose()
                error_msg = PoseStamped()
                error_msg.header.stamp = self.navigator.get_clock().now().to_msg()
                error_msg.header.frame_id = "map"
                error_msg.pose.position.x = current_pose[0] - expected_pose[0]
                error_msg.pose.position.y = current_pose[1] - expected_pose[1]
                error_msg.pose.orientation.z = current_pose[2] - expected_pose[2]
                self._error_publisher.publish(error_msg)

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("Route complete! Restarting...")
            elif result == TaskResult.CANCELED:
                print("Task route was canceled, exiting.")
                exit(1)
            elif result == TaskResult.FAILED:
                print("Task route failed!")

    def destroy_node(self):
        """Clean up after ourselves."""
        # self.navigator.destroy_node()
        self.destroy_node()


def main(args=None):
    """Launch the main function."""
    rclpy.init(args=args)

    tuner = PIDTuner()
    try:
        tuner.start_tuning()
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
