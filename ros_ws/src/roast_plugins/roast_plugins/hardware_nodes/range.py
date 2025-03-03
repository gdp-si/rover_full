from typing import Dict, Optional

import rclpy
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from sensor_msgs.msg import Range as RANGE

from roast import RobotProfile
from roast.io import Range, RangeConfig


class RangeNode(Node):
    def __init__(self):
        super().__init__("range_sensor_array_node")
        self.range: Optional[Range] = None

        self._MIN_RANGE = RangeConfig.min_range
        self._MAX_RANGE = RangeConfig.max_range
        self._FIELD_OF_VIEW = RangeConfig.field_of_view
        self._range_msg_template = RANGE()
        self._range_msg_template.radiation_type = RANGE.INFRARED
        self._range_msg_template.min_range = self._MIN_RANGE
        self._range_msg_template.max_range = self._MAX_RANGE
        self._range_msg_template.field_of_view = self._FIELD_OF_VIEW

        self._range_msgs: Dict[str, RANGE] = {}
        self._tof_publishers: Dict[str, Publisher] = {}
        self._tof_sensors_list = RobotProfile.TOF_SENSOR_ARRAY_LIST

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.range = Range()

        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        for tof in self._tof_sensors_list:
            self._tof_publishers[tof] = self.create_lifecycle_publisher(RANGE, tof, 10)
            self._range_msgs[tof] = self._range_msg_template
            self._range_msgs[tof].header.frame_id = tof

        self.timer = self.create_timer(0.02, self.publish_range_info)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        for tof in self._tof_sensors_list:
            self.destroy_publisher(self._tof_publishers[tof])

        self.destroy_timer(self.timer)
        self.timer = None
        self._tof_publishers = {}
        self.range = None

        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        for tof in self._tof_sensors_list:
            self.destroy_publisher(self._tof_publishers[tof])

        self.destroy_timer(self.timer)
        self._tof_publishers = {}
        self.timer = None
        self.range = None

        return super().on_shutdown(state)

    def _check_publishers(self) -> bool:
        for tof in self._tof_publishers.values():
            if not tof.is_activated:
                return False
        return True

    def publish_range_info(self):
        if (
            self.range is None
            or self._tof_publishers == {}
            or not self._check_publishers()
            or not self.range.is_active()
        ):
            return

        range_info = self.range.tof_update()
        for tof in self._tof_sensors_list:
            self._range_msgs[tof].range = float(range_info[tof])
            self._range_msgs[tof].header.frame_id = tof
            self._range_msgs[tof].header.stamp = self.get_clock().now().to_msg()
            self._tof_publishers[tof].publish(self._range_msgs[tof])

    def destroy_node(self):
        for tof in self._tof_sensors_list:
            self.destroy_publisher(self._tof_publishers[tof])
        self.range.destroy()
        self.destroy_timer(self.timer)
        self._tof_publishers = {}
        self.timer = None
        self.range = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        range_node = RangeNode()
        rclpy.spin(range_node)
    except KeyboardInterrupt:
        pass
    finally:
        range_node.destroy_node()


if __name__ == "__main__":
    main()
