import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger
import json
import os
import sys

class RobotPoseSaver(Node):
    def __init__(self,filename):
        super().__init__('waypoint_saver')
        self.subscription = self.create_subscription(
            Pose,
            'robot_pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.srv = self.create_service(Trigger, 'save_waypoint', self.save_waypoint_callback)

        # Define path and get filename from environment variable
        self.save_path = '/home/rover/demo_ws/src/demo_app/config'
        if not filename.endswith('.json'):
            filename += '.json'
        self.filename = filename #os.getenv('WAYPOINTS_FILE', 'test_waypoints.json')
        self.filepath = os.path.join(self.save_path, self.filename)
        self.waypoints = {}

    def pose_callback(self, msg: Pose):
        self.current_pose = {
            "position": {
                "x": msg.position.x,
                "y": msg.position.y
            },
            "orientation": {
                "x": msg.orientation.x,
                "y": msg.orientation.y,
                "z": msg.orientation.z,
                "w": msg.orientation.w
            }
        }

    def save_waypoint_callback(self, request, response):
        waypoint_name = f"waypoint{len(self.waypoints) + 1}"
        self.waypoints[waypoint_name] = self.current_pose
        
        waypoints_data = {"waypoints": self.waypoints}
        try:
            with open(self.filepath, 'w') as file:
                json.dump(waypoints_data, file, indent=4)
            self.get_logger().info(f"Waypoint {waypoint_name} saved to {self.filepath}")
            response.success = True
            response.message = f"Waypoint {waypoint_name} saved successfully"
        except Exception as e:
            self.get_logger().error(f"Failed to save waypoint: {e}")
            response.success = False
            response.message = f"Failed to save waypoint: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> <node_name> <filename>")
        sys.exit(1)
    
    filename = sys.argv[1]
    node = RobotPoseSaver(filename)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
