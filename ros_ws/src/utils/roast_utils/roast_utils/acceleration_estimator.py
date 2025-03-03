"""Robot Acceleration Estimator Node."""
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


class AccelerationEstimator(Node):
    """
    Acceleration Estimator Node for the robot.

    A utility script to estimate the actual acceleration of the robot
    given the maximum velocity of the robot.
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__("acceleration_estimator", allow_undeclared_parameters=True)

        # Declare parameters for acceleration estimator
        self.max_velocity = self.declare_parameter("max_velocity", 0.5).value
        self.max_ang_velocity = self.declare_parameter("max_ang_velocity_z", 0.5).value
        self.mode = self.declare_parameter("mode", "linear").value
        self.time_in_sec = 0
        self._msg = Odometry()

        self.odom_topic = self.get_parameter_or(
            "odometry_topic",
            Parameter("odometry_topic", Parameter.Type.STRING, "odometry"),
        ).value
        self.odom_hz = self.get_parameter_or(
            "odometry_hz", Parameter("odometry_hz", Parameter.Type.INTEGER, 10)
        ).value

        assert self.mode in [
            "linear",
            "angular",
            "both",
        ], "Invalid mode. Please choose between linear, angular or both"

        self.get_logger().info("Acceleration Estimator Node Initialized")
        self.get_logger().info(f"Subscribing to {self.odom_topic} topic")
        self._subscription = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, self.odom_hz
        )

    def _odom_callback(self, msg):
        """
        Get odometry data.

        Args:
        ----
            msg (Odometry): Odometry data

        """
        # if message velocity is 0, start the timer
        self.time_in_sec = (
            msg.header.stamp.sec if int(msg.twist.twist.linear.x) == 0 else 0
        )

        self._msg = msg

    def run(self):
        """Run the node."""
        # Based on timestamp, estimate the acceleration of the robot from the odometry data
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.mode == "linear":
                self._get_linear_acceleration()
            elif self.mode == "angular":
                self._get_angular_acceleration()
            elif self.mode == "both":
                self._get_linear_acceleration()
                self._get_angular_acceleration()

    def _get_angular_acceleration(self):
        """Get the angular acceleration of the robot."""
        if round(self._msg.twist.twist.angular.z, 1) == self.max_ang_velocity:
            time_taken = self._msg.header.stamp.sec - self.time_in_sec
            accel = (
                self.max_ang_velocity / time_taken
                if time_taken != 0
                else self.max_ang_velocity
            )
            self.get_logger().info(
                f"Time taken to reach maximum angular velocity: {time_taken} seconds"
            )
            self.get_logger().info(
                f"Angular acceleration of the robot: {accel} rad/s^2"
            )
            print("======" * 10)

    def _get_linear_acceleration(self):
        """Get the linear acceleration of the robot."""
        if round(self._msg.twist.twist.linear.x, 1) == self.max_velocity:
            time_taken = self._msg.header.stamp.sec - self.time_in_sec
            accel = (
                self.max_velocity / time_taken if time_taken != 0 else self.max_velocity
            )
            self.get_logger().info(
                f"Time taken to reach maximum velocity: {time_taken} seconds"
            )
            self.get_logger().info(f"Acceleration of the robot: {accel} m/s^2")
            print("======" * 10)


def main(args=None):
    """Launch the main function."""
    rclpy.init(args=args)
    acceleration_estimator = AccelerationEstimator()
    acceleration_estimator.run()
    acceleration_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
