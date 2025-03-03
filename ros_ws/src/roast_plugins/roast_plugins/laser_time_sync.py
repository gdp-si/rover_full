import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class MyNode(Node):
    def __init__(self):
        super().__init__("laser_time_sync_node")
        self.publisher_1_ = self.create_publisher(LaserScan, "/scan/filtered", 30)

        self.subscription_1_ = self.create_subscription(
            LaserScan, "/scan_filtered", self.subscription_callback_1, 10
        )

    def subscription_callback_1(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_1_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
