"""Ros2 script to subscriber from /publish_points and save the points in a file."""
import os

import rclpy
from ament_index_python.packages import get_resource
from geometry_msgs.msg import PointStamped
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from roast import Logger


class PatrolPointsBuilder(Node):
    """Point Builder class for patrolling application."""

    package_dir = get_resource("packages", "roast_patrol")[1]
    LOG = Logger("Patrol Points Buillder")

    def __init__(self):
        super().__init__("patrol_points_builder")
        self.subscription = self.create_subscription(
            PointStamped, "/clicked_point", self.listener_callback, 10
        )

        # Check if map_server node is running and then get the yaml_filename from the paramters
        if ("map_server", self.get_namespace()) in self.get_node_names_and_namespaces():
            if self.get_namespace() == "/":
                client = self.create_client(GetParameters, "/map_server/get_parameters")
            else:
                client = self.create_client(
                    GetParameters, f"{self.get_namespace()}/map_server/get_parameters"
                )
            ready = client.wait_for_service(timeout_sec=5)
            if not ready:
                self.LOG.CRITICAL(" Map Server Service not available")
            request = GetParameters.Request()
            request.names = ["yaml_filename"]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            response = future.result()
            if response is not None:
                yaml_filename = os.path.join(
                    self.package_dir, response.values[0].string_value
                )
            else:
                self.LOG.CRITICAL(" Map Server parameter 'yaml_filename' not available")
            map_name = yaml_filename.split("/")[-1].split(".")[0]
            self._file_path = os.path.join(
                self.package_dir,
                "share/roast_patrol/data",
                map_name,
                f"patrol_points_{map_name}.dat",
            )
        else:
            self._file_path = os.path.join(
                self.package_dir,
                "share/roast_patrol/data",
                "patrol_points_marked.dat",
            )

        # Create the file with the header
        if not os.path.exists(os.path.dirname(self._file_path)):
            os.mkdir(os.path.dirname(self._file_path))

        with open(self._file_path, "w", encoding="utf-8") as f:
            f.write("x y\n")

        self.LOG.INFO("Patrol points builder started")
        self.LOG.CRITICAL(
            f" \
            NOTE: Patrol Points recorded should be copied to the desired folder. \
            The file is available at: {self._file_path} \
            "
        )

    def listener_callback(self, msg):
        """
        Get initial pose from rviz.

        Args:
        ----
            msg (PointStamped): PointStamped message

        """
        with open(self._file_path, "a", encoding="utf-8") as file:
            print(f"{msg.point.x} {msg.point.y}")
            file.write(f"{msg.point.x:2f} {msg.point.y:2f}\n")

    def destroy_node(self):
        """Destroy node."""
        self.subscription.destroy()
        self.destroy_node()


def main(args=None):
    """Launch the main function."""
    rclpy.init(args=args)
    patrol_points_builder = PatrolPointsBuilder()
    rclpy.spin(patrol_points_builder)
    patrol_points_builder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
