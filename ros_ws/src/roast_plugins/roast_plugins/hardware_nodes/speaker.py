"""ROS Wrapper for speaker integration."""
from typing import Optional

import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.node import Subscription
from std_msgs.msg import String

from roast.io import Speaker


class SpeakerNode(Node):
    """Speaker Node."""

    def __init__(self):
        """Initialize."""
        super().__init__("speaker_node")
        self.get_logger().info("Speaker Node started")
        self._speaker_subscriber: Optional[Subscription] = None

        self._speaker: Optional[Speaker] = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configure."""

        self._speaker = Speaker()

        return super().on_configure(state)

    def _speak(self, msg: String) -> None:
        self._speaker.play(msg.data)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activate."""
        self._speaker_subscriber = self.create_subscription(
            String, "speak", self._speak, 10
        )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Deactivate."""
        self.destroy_subscription(self._speaker_subscriber)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Cleanup."""
        self.destroy_subscription(self._speaker_subscriber)

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Shutdown."""
        self.destroy_subscription(self._speaker_subscriber)

        return super().on_shutdown(state)

    def destroy_node(self) -> None:
        """Destroy node."""
        self.destroy_subscription(self._speaker_subscriber)
        super().destroy_node()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
