"""Odometry subscriber node to write the odometry data to a file."""
import os
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from roast.glogging import Logger
from roast.robot import RobotParameters
from roast.settings import DATA_PATH


class OdometryPoseHandler(Node):
    """Class to handle odometry data."""

    LOG = Logger("Odometry Pose Handler")

    def __init__(self):
        self._count: int = 0
        self._subscriber: Optional[Subscription] = None
        self._timer: Optional[Timer] = None
        super().__init__("odometry_pose_handler")

        self._data = Odometry()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        if not os.path.exists(DATA_PATH):
            os.makedirs(DATA_PATH)
        self._save_path = os.path.join(DATA_PATH, "odometry.dat")
        self._last_path = os.path.join(DATA_PATH, "initial_pose.dat")

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Odometry pose publisher is activated.")
        self._subscriber = self.create_subscription(
            Odometry, "odometry/filtered", self._callback, 1
        )

        self._timer = self.create_timer(1, self._timer_callback)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Odometry Pose publisher is deactivated")
        self.destroy_subscription(self._subscriber)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_subscription(self._subscriber)

        self.LOG.INFO("on_cleanup() is called")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self._timer)
        self.destroy_subscription(self._subscriber)

        self.LOG.INFO("on_shutdown() is called")

        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self) -> None:
        with open(self._last_path, "w", encoding="utf-8") as f:
            data = f"""timestamp: {self._data.header.stamp.sec}.{self._data.header.stamp.nanosec} \
position: {self._data.pose.pose.position.x:2f} {self._data.pose.pose.position.y:2f} \
orientation: {self._data.pose.pose.orientation.x:2f} {self._data.pose.pose.orientation.y:2f} {self._data.pose.pose.orientation.z:2f} {self._data.pose.pose.orientation.w:2f}"""  # noqa: E501
            f.write(data)

        self.LOG.INFO(f"File written to: {self._last_path}")

        self._subscriber.destroy()
        self._timer.destroy()

        super().destroy_node()

    def _callback(self, msg):
        """Callback function for odometry data."""
        self._data = msg

    def _timer_callback(self):
        """Callback function for timer."""
        if self._subscriber is None:
            self.LOG.INFO(
                "OdometryPoseHandler is not active. Messages are not written to the file."
            )
            return
        else:
            self.LOG.DEBUG(
                "OdometryPoseHandler is active. Data will be written to file"
            )

        self.LOG.DEBUG("Writing odometry data to file...")
        with open(self._save_path, "w") as f:
            data = f"""timestamp: {self._data.header.stamp.sec}.{self._data.header.stamp.nanosec} \
position: {self._data.pose.pose.position.x:2f} {self._data.pose.pose.position.y:2f} \
orientation: {self._data.pose.pose.orientation.x:2f} {self._data.pose.pose.orientation.y:2f} {self._data.pose.pose.orientation.z:2f} {self._data.pose.pose.orientation.w:2f}"""  # noqa: E501
            f.write(data)


def main(args=None):
    rclpy.init(args=args)

    try:
        odometry_pose_handler = OdometryPoseHandler()
        rclpy.spin(odometry_pose_handler)
    finally:
        odometry_pose_handler.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
