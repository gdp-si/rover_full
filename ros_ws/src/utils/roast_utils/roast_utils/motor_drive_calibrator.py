"""Test cmd velocity commands to the motors."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter


class MotorDriveTester(Node):
    """Test cmd velocity commands to the motors."""

    def __init__(self):
        """Initialize the node with default values."""
        super().__init__("motor_drive_tester", allow_undeclared_parameters=True)

        self._magnitude = self.declare_parameter("pulse_magnitude", 0.5).value
        self._hz = self.declare_parameter("pulse_freq", 8.0).value
        self._mode = self.declare_parameter("mode", "linear").value

        assert self._mode in [
            "linear",
            "angular",
        ], "Invalid mode. Please choose between linear or angular"

        self._cmd_vel_topic = self.get_parameter_or(
            "cmd_vel_topic",
            Parameter("cmd_vel_topic", Parameter.Type.STRING, "cmd_vel"),
        ).value

        self.get_logger().info("Motor Drive Tester Node Initialized")
        self.get_logger().info(f"Publishing to {self._cmd_vel_topic} topic")

        self.publisher = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Send a test velocity command to the motors."""
        # Send sine wave in the x direction
        msg = Twist()
        if self._mode == "linear":
            msg.linear.x = self._magnitude * (
                math.sin(self._hz * self.get_clock().now().nanoseconds / 1000000000)
            )
        else:
            msg.angular.z = self._magnitude * (
                math.sin(self._hz * self.get_clock().now().nanoseconds / 1000000000)
            )
        self.publisher.publish(msg)


def main(args=None):
    """Start the node."""
    rclpy.init(args=args)

    node = MotorDriveTester()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
