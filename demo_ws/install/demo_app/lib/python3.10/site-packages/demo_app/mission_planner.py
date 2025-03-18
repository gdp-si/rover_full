import json
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
from action_msgs.msg import GoalStatus

class MissionPlanner(Node):
    def __init__(self, mission_file):
        super().__init__('mission_planner')
        self.mission_file = mission_file
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.load_mission()
        self.get_logger().info(f"Loaded mission from {self.mission_file}: {self.tasks}")
    
    def load_json(self, filename):
        try:
            with open(filename, 'r') as file:
                return json.load(file)
        except Exception as e:
            self.get_logger().error(f"Error reading {filename}: {e}")
            sys.exit(1)
    
    def load_mission(self):
        mission_data = self.load_json(self.mission_file)
        wp_file = mission_data.get("waypoint_file", "waypoints.json")
        waypoint_file = os.path.join('/home/rover/demo_ws/src/demo_app/config', wp_file)
        if not waypoint_file.endswith('.json'):
            waypoint_file += '.json'
        waypoints_data = self.load_json(waypoint_file)
        self.waypoints = waypoints_data.get("waypoints", {})
        self.tasks = mission_data.get("mission", {}).get("tasks", [])
    
    def execute_mission(self):
        self.get_logger().info("Starting mission execution")
        for index, task in enumerate(self.tasks, start=1):
            self.get_logger().info(f"Executing task {index}/{len(self.tasks)}: {task}")
            if task.get("task") == "goto":
                waypoint_name = task.get("waypoint")
                waypoint_info = self.waypoints.get(waypoint_name, None)
                if waypoint_info:
                    self.navigate_to_pose(waypoint_name, waypoint_info)
                else:
                    self.get_logger().warn(f"Waypoint {waypoint_name} not found")
            elif task.get("task") == "delay":
                delay_seconds = task.get("seconds", 0)
                self.get_logger().info(f"Delaying for {delay_seconds} seconds")
                time.sleep(delay_seconds)
        self.get_logger().info("Mission execution completed")
    
    def navigate_to_pose(self, waypoint_name, waypoint_info):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint_info["position"]["x"]
        goal_msg.pose.pose.position.y = waypoint_info["position"]["y"]
        goal_msg.pose.pose.orientation.x = waypoint_info["orientation"]["x"]
        goal_msg.pose.pose.orientation.y = waypoint_info["orientation"]["y"]
        goal_msg.pose.pose.orientation.z = waypoint_info["orientation"]["z"]
        goal_msg.pose.pose.orientation.w = waypoint_info["orientation"]["w"]

        self.client.wait_for_server()
        self.get_logger().info(f"Sending goal to {waypoint_name}")
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to {waypoint_name} was rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED: # and result.result and result.result.success:
            self.get_logger().info(f"Successfully reached {waypoint_name}")
        else:
            self.get_logger().warn(f"Failed to reach {waypoint_name}")
            
            
def main(args=None):
    rclpy.init()
    if len(sys.argv) < 2:
        print("Usage: python read_mission.py <mission_file>")
        sys.exit(1)
    filename = os.path.join('/home/rover/demo_ws/src/demo_app/config', sys.argv[1])
    if not filename.endswith('.json'):
        filename += '.json'
    node = MissionPlanner(filename)
    node.execute_mission()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
