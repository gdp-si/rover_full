"""ROS Wrapper for Display integration."""
from typing import Optional

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.node import Subscription
from roast_interfaces.msg import DisplayMode

from roast.ui import Display


class DisplayNode(Node):
    """Display Node."""

    def __init__(self):
        """Initialize."""
        super().__init__("display_node")
        self.get_logger().info("Display Node started")
        self._display_subscriber: Optional[Subscription] = None

        self._display: Optional[Display] = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure."""

        self._display = Display()
        self._display.display("BLINK")

        return super().on_configure(state)

    def _speak(self, msg: DisplayMode) -> None:
        mode = ""

        if msg.display_mode == msg.MODE_BLINK_THRICE:
            mode = "BLINK_THRICE"
        elif msg.display_mode == msg.MODE_PATROL:
            mode = "PATROL"
        elif msg.display_mode == msg.MODE_BOOTUP:
            mode = "BOOTUP"
        elif msg.display_mode == msg.MODE_THREAT_TRACKING_ON:
            mode = "THREAT_TRACKING_ON"
        elif msg.display_mode == msg.MODE_THREAT_TRACKING_OFF:
            mode = "THREAT_TRACKING_OFF"
        elif msg.display_mode == msg.MODE_CUSTOM:
            mode = "CUSTOM"
        else:
            mode = "BLINK"

        self._display.display(mode)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate."""
        self._display_subscriber = self.create_subscription(
            DisplayMode, "display_mode", self._speak, 10
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate."""
        self.destroy_subscription(self._display_subscriber)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup."""
        self.destroy_subscription(self._display_subscriber)

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown."""
        self.destroy_subscription(self._display_subscriber)

        return super().on_shutdown(state)

    def destroy_node(self) -> None:
        """Destroy node."""
        self.destroy_subscription(self._display_subscriber)
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = DisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
