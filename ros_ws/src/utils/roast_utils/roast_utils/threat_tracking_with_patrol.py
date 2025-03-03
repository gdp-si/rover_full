"""
Target Tracking for Roast AI.
Note that this code is a legacy code and will be replaced by the new data handler.
"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.node import Node
from roast_plugins import ROAST_PROFILE
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker

from roast.glogging import Logger

PKG_DIRECTORY = get_package_share_directory("roast_threat_traker")


class ThreatTracker(Node):
    """Threat Tracker which sends the tf position and pose of the target"""

    LOG = Logger("Target Tracker & Patrol")

    _threat_found = False
    behavior_tree = os.path.join(PKG_DIRECTORY, "behavior_trees/threat_tracker.xml")

    def __init__(self, debug: bool = False, patrol_config: str = None) -> None:
        super().__init__("target_tracker")
        self.debug = debug
        self.NEXT_GOAL = True

        # Declare Parameters
        self.declare_parameter("namespace", "")
        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )

        # FIXME: Upgrade the system to use new data handler
        if isinstance(patrol_config, str):
            pkg_share = get_package_share_directory("roast_utils")
            patrol_config = os.path.join(pkg_share, "config", patrol_config)
            if os.path.isfile(patrol_config):
                self.patrol_config = self.load_patrol_config(patrol_config)
            else:
                raise FileNotFoundError(
                    f"Patrol config file not found: {patrol_config}"
                )
        else:
            raise ValueError(
                "Patrol config file does not exist. Try setting absolute path"
            )

        self.patrol_points = []
        self.patrol_index = self.patrol_config["patrol_id"]
        self.patrol_timer = None
        self.patrol_timer_duration = Duration(seconds=3)
        self.patrol_timer_remaining = self.patrol_timer_duration

        for point in self.patrol_config["patrol_points"]:
            self.patrol_points.append(tuple(point))

        # Create subscribers for camera
        self._threat_pose_subscriber = self.create_subscription(
            PoseStamped,
            "threat_pose/safe_pose",
            self._threat_safe_pose_callback,
            ROAST_PROFILE,
        )

        self._threat_pose_raw_subscriber = self.create_subscription(
            PoseStamped,
            "threat_pose",
            self._threat_pose_raw_callback,
            ROAST_PROFILE,
        )

        self.threat_found_subscriber = self.create_subscription(
            Bool, "/threat_found", self._threat_found_callback, ROAST_PROFILE
        )

        # If debugging mode, add visualization markers
        if self.debug:
            self._target_pose_marker_publisher = self.create_publisher(
                Marker, "/target_pose_marker", 1
            )

            self._target_marker = Marker()

            self._target_marker.header.frame_id = "target"
            self._target_marker.header.stamp = self.get_clock().now().to_msg()
            self._target_marker.type = Marker.CYLINDER
            self._target_marker.scale.x = 1.0
            self._target_marker.scale.y = 1.0
            self._target_marker.scale.z = 1.0
            self._target_marker.color.r = 1.0
            self._target_marker.color.g = 0.0
            self._target_marker.color.b = 0.0
            self._target_marker.color.a = 1.0
            # self._target_marker.lifetime = rclpy.Duration(5)
            self._target_marker.frame_locked = False

        # Create Broadcaster
        self._transformation_bridge = TransformBroadcaster(self)

        # Create TF Listener
        self._tf_buffer = Buffer()
        self._listener = TransformListener(self._tf_buffer, self)

        # Setup messages
        self._safe_target_pose = PoseStamped()
        self._safe_target_pose.header.frame_id = "camera"
        self._safe_target_pose.header.stamp = self.get_clock().now().to_msg()
        self._safe_target_pose.pose.orientation.w = 1.0

        self.point = PoseStamped()
        self.point.pose.orientation.w = 1.0

        # Compute target pose with respect to the threat -> odom frame
        self._target_pose = PoseStamped()
        self._target_pose_raw = PoseStamped()

        # Setup Navigator
        self._navigator = BasicNavigator()
        # self._navigator.waitUntilNav2Active(localizer="")

        # Setup timer
        self._timer = self.create_timer(1, self._timer_callback)

        self.LOG.INFO("Target Tracker initialized")

    def load_patrol_config(self, config_file: str) -> dict:
        """Loads a patrol configuration from a file"""

        with open(config_file, "r", encoding="utf-8") as f:
            config = json.load(f)

        return config

    def _threat_safe_pose_callback(self, msg: PoseStamped) -> None:
        """Threat pose callback"""
        self._safe_target_pose = msg

    def _threat_pose_raw_callback(self, msg: PoseStamped) -> None:
        """Threat pose raw callback"""
        self._target_pose_raw = msg

    def _threat_found_callback(self, msg: Bool) -> None:
        """Threat found callback"""
        self._threat_found = msg.data

    def _timer_callback(self) -> None:
        """Timer callback"""

        # Send target broadcaster
        self._target_broadcaster()  # Send target pose from threat to camera frame
        self._transformation_listener()  # Compute the transformation from the threat to odom frame

        if self._threat_found and self._target_pose != PoseStamped():
            self._navigator.cancelTask()
            self.patrol_points.insert(0, self.point)
            self.patrol_points = list(dict.fromkeys(self.patrol_points))
            self.LOG.INFO("THREAT FOUND - SENDING TARGET")
            # self._navigator.cancelTask()
            self._navigator.goToPose(
                self._target_pose, behavior_tree=self.behavior_tree
            )

            # If debug, publish the marker
            if self.debug:
                self._target_marker.header.stamp = self.get_clock().now().to_msg()
                self._target_marker.pose = self._target_pose_raw.pose
                self._target_pose_marker_publisher.publish(self._target_marker)
        else:
            if self.NEXT_GOAL:
                self.point = self.patrol_points.pop(0)
                self.patrol_points.append(self.point)
                self.LOG.INFO(f"THREAT NOT FOUND - SENDING PATROL {self.point}")
                self.NEXT_GOAL = False

            print(f"{self.patrol_points}")
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = (
                self._navigator.get_clock().now().to_msg()
            )  # Keep the timestamp of nav2
            pose.pose.position.x = float(self.point[0])
            pose.pose.position.y = float(self.point[1])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self._navigator.goToPose(pose)

        while not self._navigator.isTaskComplete():
            # print("Moving towards goal")
            print(self._threat_found)
            # self._navigator.clearGlobalCostmap()
            result = self._navigator.getResult()
            if result == TaskResult.SUCCEEDED or self._threat_found:
                self.LOG.INFO("Target reached")
                break
        else:
            self.NEXT_GOAL = True

    def _transformation_listener(self) -> None:
        """Get the transformation from the threat to the odom frame"""

        from_frame = "odom"
        to_frame = "target"

        # Compute Transformation
        try:
            t = self._tf_buffer.lookup_transform(
                from_frame, to_frame, rclpy.time.Time()
            )
            self._target_pose.header.stamp = t.header.stamp
            self._target_pose.header.frame_id = t.header.frame_id
            self._target_pose.pose.position.x = t.transform.translation.x
            self._target_pose.pose.position.y = t.transform.translation.y
            self._target_pose.pose.position.z = t.transform.translation.z
            self._target_pose.pose.orientation.x = t.transform.rotation.x
            self._target_pose.pose.orientation.y = t.transform.rotation.y
            self._target_pose.pose.orientation.z = t.transform.rotation.z
            self._target_pose.pose.orientation.w = t.transform.rotation.w
        except Exception:
            self._target_pose = PoseStamped()

    def _target_broadcaster(self) -> None:
        """Target pose broadcaster"""
        t = TransformStamped()
        t.header.frame_id = "camera"
        t.header.stamp = self._safe_target_pose.header.stamp

        t.child_frame_id = "target"
        t.transform.translation.x = self._safe_target_pose.pose.position.x
        t.transform.translation.y = self._safe_target_pose.pose.position.y
        t.transform.translation.z = self._safe_target_pose.pose.position.z
        t.transform.rotation.x = self._safe_target_pose.pose.orientation.x
        t.transform.rotation.y = self._safe_target_pose.pose.orientation.y
        t.transform.rotation.z = self._safe_target_pose.pose.orientation.z
        t.transform.rotation.w = self._safe_target_pose.pose.orientation.w

        self._transformation_bridge.sendTransform(t)

    def destroy_node(self):
        """Destroy node"""
        self._navigator.destroy_node()
        super().destroy_node()
        self.LOG.INFO("Target Tracker destroyed")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    threat_tracker = ThreatTracker(debug=True, patrol_config="patrol_points.json")

    rclpy.spin(threat_tracker)

    threat_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
