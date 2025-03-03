import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from roast_interfaces.msg import PeopleGroup
from std_msgs.msg import Header


def generate_pose(x, y, z) -> PoseStamped:
    return PoseStamped(
        header=Header(
            frame_id="map",
            stamp=Time(
                sec=0,
                nanosec=0,
            ),
        ),
        pose=Pose(
            position=Point(
                x=float(x),
                y=float(y),
                z=float(z),
            ),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=0.0,
                w=1.0,
            ),
        ),
    )


class DummyPeopleInfoPublisher(Node):
    def __init__(self):
        super().__init__("dummy_people_info_publisher")
        self.group_info_pub = self.create_publisher(PeopleGroup, "people_info", 10)
        self.timer = self.create_timer(1 / 10.0, self.publish_people_info)
        self.get_logger().info("Dummy People Info Publisher has been started")

    def publish_people_info(self):
        group_info = PeopleGroup(
            frame_id="map",
            stamp=Time(
                sec=0,
                nanosec=0,
            ),
            group_detected=True,
            group_poses=[
                generate_pose(-2.0, 11.0, 0.0),
                generate_pose(-3.0, 11.0, 0.0),
                generate_pose(-1.0, 11.0, 0.0),
            ],
        )
        self.group_info_pub.publish(group_info)


def main(args=None):
    rclpy.init(args=args)
    dummy_people_info_publisher = DummyPeopleInfoPublisher()
    rclpy.spin(dummy_people_info_publisher)
    dummy_people_info_publisher.destroy_node()


if __name__ == "__main__":
    main()
