"""
ROS Wrapper for motor control of the roast

- Skid Steering Mechanism (Virtual Differential Drive)
- PID Control.
"""

from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from rclpy.parameter import Parameter
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from roast_interfaces.msg import EncoderData
from roast_plugins import ROAST_PROFILE
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from roast.glogging import Logger
from roast.io import MotorControl
from roast.io.configs import MotorDriveConfig
from roast.robot import RobotParameters
from roast.robot.control.kinematic_model import Robot
from roast.utils.filters import filter_outliers


class RobotControl(Node):
    """Robot Control for skid steering robot model

    Args:
        Node (rclpy.Node): Node Object
    """

    LOG = Logger(_module_name="Robot Control", _level=20)

    current_time = 0
    last_time = 0
    left_data = (0.0, 0.0)
    right_data = (0.0, 0.0)

    def __init__(self):
        super().__init__("robot_control")
        self._cmd_vel: Optional[Subscription] = None
        self._odometry: Optional[Subscription] = None
        self._encoder_left: Optional[Publisher] = None
        self._encoder_right: Optional[Publisher] = None
        self._left_status_publisher: Optional[Publisher] = None
        self._right_status_publisher: Optional[Publisher] = None
        self.timer: Optional[Timer] = None
        self.status_timer: Optional[Timer] = None

        self.Kp = None
        self.Ki = None
        self.Kd = None
        self._msg = None
        self.current_pose = None
        self.encoder_left = None
        self.encoder_right = None
        self.ROBOT = None
        self.LEFT_MOTOR = None
        self.RIGHT_MOTOR = None
        self.ROBOT = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Configuring robot control node")

        # Setup messages
        self._msg = Twist()
        self.current_pose = Pose()
        self.encoder_left = EncoderData()
        self.encoder_right = EncoderData()

        self.ROBOT = Robot(
            wheel_radius=RobotParameters.wheel_diameter / 2,
            robot_length=RobotParameters.robot_length,
            robot_width=RobotParameters.robot_width,
        )

        # Robot motor control
        self.LEFT_MOTOR = MotorControl("left", MotorDriveConfig.PARAMS_LEFT)
        self.RIGHT_MOTOR = MotorControl("right", MotorDriveConfig.PARAMS_RIGHT)
        self.LEFT_MOTOR.enable_motor()
        self.RIGHT_MOTOR.enable_motor()

        # Encoders filtering
        self._window_size = (
            self.declare_parameter(
                "encoders_window_size", RobotParameters.encoders_window_size
            )
            .get_parameter_value()
            .integer_value
        )
        self._left_encoder_data = [0.0] * self._window_size
        self._right_encoder_data = [0.0] * self._window_size
        self._left_encoder_data_index = 0
        self._right_encoder_data_index = 0

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Robot control is activated.")

        self._reset_encoder_service = self.create_service(
            SetBool,
            f"{self.get_name()}/reset_encoders",
            self._reset_encoder_callback,
            qos_profile=ROAST_PROFILE,
        )

        # Setup publishers
        self._encoder_left = self.create_lifecycle_publisher(
            EncoderData, "encoder_left/data", qos_profile=ROAST_PROFILE
        )
        self._encoder_right = self.create_lifecycle_publisher(
            EncoderData, "encoder_right/data", qos_profile=ROAST_PROFILE
        )
        self._left_status_publisher = self.create_lifecycle_publisher(
            Bool,
            "hardware_info/status/primary/left_motor",
            qos_profile=ROAST_PROFILE,
        )
        self._right_status_publisher = self.create_lifecycle_publisher(
            Bool,
            "hardware_info/status/primary/right_motor",
            qos_profile=ROAST_PROFILE,
        )

        # setup timer
        self.timer = self.create_timer(
            1 / RobotParameters.PUBLISH_RATE["encoder"], self._publish_encoder
        )

        status_timer_period = 1 / RobotParameters.PUBLISH_RATE["status"]  # seconds
        self.status_timer = self.create_timer(
            status_timer_period, self.status_timer_callback
        )

        self._cmd_vel = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, qos_profile=ROAST_PROFILE
        )

        self._odometry = self.create_subscription(
            Odometry, "odometry", self.odometry_callback, qos_profile=ROAST_PROFILE
        )

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.LOG.INFO("Robot control is deactivated")
        self.destroy_subscription(self._cmd_vel)
        self.destroy_subscription(self._odometry)

        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self._encoder_left)
        self.destroy_publisher(self._encoder_right)
        self.destroy_publisher(self._left_status_publisher)
        self.destroy_publisher(self._right_status_publisher)
        self.destroy_timer(self.timer)
        self.destroy_timer(self.status_timer)
        self.destroy_subscription(self._cmd_vel)
        self.destroy_subscription(self._odometry)
        self.destroy_service(self._reset_encoder_service)

        self.LOG.INFO("on_cleanup() is called")

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self._encoder_left)
        self.destroy_publisher(self._encoder_right)
        self.destroy_publisher(self._left_status_publisher)
        self.destroy_publisher(self._right_status_publisher)
        self.destroy_timer(self.timer)
        self.destroy_timer(self.status_timer)
        self.destroy_subscription(self._cmd_vel)
        self.destroy_subscription(self._odometry)
        self.destroy_service(self._reset_encoder_service)

        self.LOG.INFO("on_shutdown() is called")

        return TransitionCallbackReturn.SUCCESS

    def _reset_encoder_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ):
        if request.data:
            self.LEFT_MOTOR.clear_encoder_values()
            self.RIGHT_MOTOR.clear_encoder_values()

        response.success = True

        return response

    def _parameter_callback(self, params):
        """Parameter Callback"""

        for param in params:
            if param.name == "Kp" and param.type_ == Parameter.Type.DOUBLE:
                self.Kp = param.value
            elif param.name == "Ki" and param.type_ == Parameter.Type.DOUBLE:
                self.Ki = param.value
            elif param.name == "Kd" and param.type_ == Parameter.Type.DOUBLE:
                self.Kd = param.value

        self.LOG.INFO(
            f"Updated PID parameters: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}"
        )
        return SetParametersResult(successful=True)

    def odometry_callback(self, msg):
        """Odometry Callback

        Args:
            msg (Odometry): Odometry message for the robot
        """
        self.current_pose = msg.pose.pose

    def cmd_vel_callback(self, msg):
        """Command velocity Callback

        Args:
            msg (Twist): Twist message for the robot
        """
        self._msg = msg
        self.current_time = self.get_clock().now().nanoseconds / 1000000000

        def _map(value, in_min, in_max, out_min, out_max):
            return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        self._msg.linear.x = _map(
            self._msg.linear.x,
            -1,
            1,
            -RobotParameters.robot_velocity_limit,
            RobotParameters.robot_velocity_limit,
        )
        self._msg.linear.y = _map(
            self._msg.linear.y,
            -1,
            1,
            -RobotParameters.robot_velocity_limit,
            RobotParameters.robot_velocity_limit,
        )

        # Calculate desired robot position
        # TODO: check this function if still correct
        desired = self.ROBOT.calculate_position(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            self.last_time - self.current_time,
        )

        # Calculate the error
        error = (
            desired[0] - self.current_pose.position.x,
            desired[1] - self.current_pose.position.y,
            desired[2] - self.current_pose.orientation.z,
        )

        (
            front_left,
            front_right,
            back_left,
            back_right,
        ) = self.ROBOT.calculate_wheel_rates(
            self._msg.linear.x, self._msg.linear.y, self._msg.angular.z
        )

        # Update the motor speeds
        # print(front_left,front_right,back_left,back_right)
        left = (front_left + back_left) / 2
        right = (front_right + back_right) / 2
        self.LEFT_MOTOR.update(left, right)
        # self.RIGHT_MOTOR.update((front_right + back_right) / 2)

        # Update last time
        self.last_time = self.current_time

    def _publish_encoder(self):
        """Publish encoder data"""
        if (
            self.LEFT_MOTOR is None
            or self.RIGHT_MOTOR is None
            or not self._encoder_left.is_activated
            or not self._encoder_right.is_activated
        ):
            return

        self.left_data = self.LEFT_MOTOR.lpulse_counter()
        self.right_data = self.RIGHT_MOTOR.rpulse_counter()

        self._left_encoder_data[self._left_encoder_data_index] = self.left_data
        self._right_encoder_data[self._right_encoder_data_index] = self.right_data

        if self._left_encoder_data_index == self._window_size - 1:
            self._left_encoder_data_index = 0
        else:
            self._left_encoder_data_index += 1

        if self._right_encoder_data_index == self._window_size - 1:
            self._right_encoder_data_index = 0
        else:
            self._right_encoder_data_index += 1

        self.encoder_left.data = filter_outliers(
            self._left_encoder_data, window_size=self._window_size
        )[0]
        self.encoder_right.data = filter_outliers(
            self._right_encoder_data, window_size=self._window_size
        )[0]

        # TODO: Add separate thread for each encoder data
        self.encoder_right.header.stamp = self.encoder_left.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self._encoder_left.publish(self.encoder_left)
        self._encoder_right.publish(self.encoder_right)

    def status_timer_callback(self):
        """Callback function for timer"""
        if (
            self.LEFT_MOTOR is None
            or self.RIGHT_MOTOR is None
            or not self._left_status_publisher.is_activated
            or not self._right_status_publisher.is_activated
        ):
            return
        self._left_status_publisher.publish(Bool(data=self.LEFT_MOTOR.is_active()))
        self._right_status_publisher.publish(Bool(data=self.RIGHT_MOTOR.is_active()))

    def destroy_node(self):
        """Destroy ROS node"""
        self.LOG.INFO("Destroying Robot Control Node")
        self.LEFT_MOTOR.update()  # To stop the motors
        self.RIGHT_MOTOR.update()
        self.LEFT_MOTOR.destroy()
        self.RIGHT_MOTOR.destroy()

        self.destroy_publisher(self._encoder_left)
        self.destroy_publisher(self._encoder_right)
        self.destroy_publisher(self._left_status_publisher)
        self.destroy_publisher(self._right_status_publisher)
        self.destroy_publisher(self._cmd_vel)
        self.destroy_publisher(self._odometry)

        self.destroy_timer(self.timer)
        self.destroy_timer(self.status_timer)

        self.destroy_service(self._reset_encoder_service)

        super().destroy_node()


def main():
    """Main function"""
    rclpy.init(args=None)

    try:
        robot_control = RobotControl()
        rclpy.spin(robot_control)
    except KeyboardInterrupt:
        pass
    finally:
        robot_control.destroy_node()


if __name__ == "__main__":
    main()
