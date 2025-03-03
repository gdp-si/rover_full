"""ROS Wrapper for Threat Spray Module.

Usage:

ros2 service call /threat_spray_service roast_interfaces/srv/ThreatSpray "{angle: 0.0, spray_duration: 2.0, spray_interval: 0.0, start_spray: true}"
ros2 topic pub /led_mode roast_interfaces/msg/LedMode "{led_mode: 0}"
"""
import time

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from roast_interfaces.msg import LedMode
from roast_interfaces.srv import SprayModule
from roast_plugins import ROAST_PROFILE

from roast.glogging import Logger
from roast.io import ThreatSprayModule


class ThreatSprayNode(Node):
    """Threat Spraying Service and LED controller for ROS.

    Args:
        Node (rclpy.Node): RCLPY Node
    """

    def __init__(self):
        """Initialize Threat Spray Service."""
        super().__init__("threat_spray_service")
        self._logger = Logger("Threat Spray Module")
        self._logger.INFO("Starting Threat Spray Service")

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._threat_spray_module = ThreatSprayModule()
        self._led_freeze = False

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._logger.INFO("Creating Threat Spray Service")

        self._threat_spray_service = self.create_service(
            SprayModule,
            "threat_spray_service",
            self._threat_spray_callback,
            qos_profile=ROAST_PROFILE,
        )
        # Subscribe to LED Mode
        self._led_mode_subscriber = self.create_subscription(
            LedMode, "led_mode", self._led_mode_callback, qos_profile=ROAST_PROFILE
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._logger.INFO("Destroying Threat Spray Service")
        self.destroy_service(self._threat_spray_service)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._logger.INFO("Cleaning up Threat Spray Service")
        self.destroy_service(self._threat_spray_service)

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._logger.INFO("Shutting down Threat Spray Service")
        self.destroy_service(self._threat_spray_service)

        return TransitionCallbackReturn.SUCCESS

    def _threat_spray_callback(self, request, response):
        """Threat Spray Callback."""
        self._logger.DEBUG("Threat Spray Service Callback")
        if request.start_spray:
            duration = request.spray_duration
            interval = request.spray_interval

            self._led_freeze = True

            self._threat_spray_module.retraction_timeout = duration
            self._threat_spray_module.update(command="threat")

            time.sleep(duration)

            response.success = True
            self._led_freeze = False
        else:
            response.success = False

        return response

    def _led_mode_callback(self, msg):
        """LED Mode Callback."""

        if self._led_freeze:
            return

        self._logger.DEBUG("LED Mode Callback")
        if msg.led_mode == msg.NAVIGATION_SETUP:
            self._threat_spray_module.update(command="move")
        elif msg.led_mode == msg.BATTERY_LOW:
            self._threat_spray_module.update(command="stop")
        elif msg.led_mode == msg.BATTERY_CRITICAL:
            self._threat_spray_module.update(command="off")
        elif msg.led_mode == msg.THREAT_DETECTED:
            self._threat_spray_module.update(command="threat")


def main(args=None):
    """Main function for Threat Spray Service."""
    rclpy.init(args=args)
    threat_spray_service = ThreatSprayNode()
    rclpy.spin(threat_spray_service)
    threat_spray_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
