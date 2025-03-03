"""Set initial pose for the robot in ROS2."""
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.lifecycle import LifecycleNode, Publisher, State, TransitionCallbackReturn
from rclpy.timer import Timer
from std_srvs.srv import Trigger

from roast.glogging import Logger
from roast.utils import robot_initial_pose


class SetInitialPose(LifecycleNode):
    """Class definition to set initial pose for the robot in ROS2."""

    LOG = Logger("Initial Pose Handler")

    _pose = PoseWithCovarianceStamped()
    _pose.header.frame_id = "map"
    _pose.pose.pose.position.x = 0.0
    _pose.pose.pose.position.y = 0.0
    _pose.pose.pose.position.z = 0.0
    _pose.pose.pose.orientation.w = 1.0

    def __init__(self) -> None:
        super().__init__("initial_pose_handler")

        self._initial_pose_publisher: Optional[Publisher] = None
        self._timer: Optional[Timer] = None
        self._elapsed_time = 0.0

        # Setup timer
        self.LOG.INFO("Setup initial pose")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        (
            self._pose.pose.pose.position.x,
            self._pose.pose.pose.position.y,
            self._pose.pose.pose.position.z,
            self._pose.pose.pose.orientation.x,
            self._pose.pose.pose.orientation.y,
            self._pose.pose.pose.orientation.z,
            self._pose.pose.pose.orientation.w,
        ) = robot_initial_pose(with_quaternion=True)
        self.LOG.INFO("Configuring initial pose")

        # Timeout parameter
        self._timeout = self.declare_parameter("initial_pose_timeout", 1.0).value

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Activating initial pose handler")
        self._initial_pose_publisher = self.create_lifecycle_publisher(
            PoseWithCovarianceStamped, "initialpose", 10
        )
        self._timer = self.create_timer(0.03, self._timer_callback)

        self._request_initial_pose = self.create_service(
            Trigger, "request_initial_pose", self._request_initial_pose_callback
        )

        self._elapsed_time = 0.0
        self._has_timer_expired = False

        return super().on_activate(state)

    def _request_initial_pose_callback(self, request, response):
        """Callback function for the service request."""
        self._has_timer_expired = False
        self._elapsed_time = 0.0

        response.success = True
        response.message = "Initial pose set"

        return response

    def on_deactivate(self, state: int) -> TransitionCallbackReturn:
        self.LOG.INFO("Deactivating initial pose handler")
        self.destroy_publisher(self._initial_pose_publisher)
        self.destroy_timer(self._timer)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Deactivating initial pose handler")
        self.destroy_publisher(self._initial_pose_publisher)
        self.destroy_timer(self._timer)

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Shuting down initial pose handler")
        self._initial_pose_publisher.destroy()
        self._timer.destroy()

        return super().on_shutdown(state)

    def _timer_callback(self) -> None:
        """Timer callback function."""

        if (
            self._initial_pose_publisher is not None
            and self._initial_pose_publisher.is_activated
            and not self._has_timer_expired
        ):
            self._pose.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_publisher.publish(self._pose)

            # If elapsed time is greater than 10 seconds, stop publishing
            self._elapsed_time += 0.03
            if self._elapsed_time > self._timeout:
                self._has_timer_expired = True

    def destroy(self) -> None:
        """Destroy node."""
        self.LOG.INFO("Destroying `INITIAL POSE HANDLER` node")
        self._initial_pose_publisher.destroy()
        self._timer.destroy()
        super().destroy_node()


def main(args=None):
    """Start the main function."""
    rclpy.init(args=args)

    try:
        initial_pose = SetInitialPose()
        rclpy.spin(initial_pose)
    finally:
        initial_pose.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
