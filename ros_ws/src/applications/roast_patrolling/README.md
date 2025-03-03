# Roast Patrolling

This repository contains the code for the Roast Patrol application. The Roast Patrol application is a ROS2 package that uses ROS2 Navigation2 to navigate a robot to a series of waypoints. The robot will try to constantly patrol the waypoints, if the navigaiton is interrupted, the robot will try to resume the patrol from the current known position. The robot will also try to avoid obstacles in its path.

### Two modes of operation

- Sequential: The robot will navigate to each waypoint in the order they are specified in the waypoint file.
- Random: The robot will navigate to each waypoint in a random order.

### Usage

To setup the waypoints for the patrolling algorithm, update the launch file `patrolling.launch.py`. To test the patrolling algorithm, run the following command:

```bash
# To start the patrolling action server
ros2 launch roast_patrolling patrolling.launch.py

# To run in sequential mode
ros2 action send_goal /patrol  roast_interfaces/action/PatrolAction "{start_patrol: true, patrol_type: sequential, behavior_tree:}"

# To run in random mode
ros2 action send_goal /patrol  roast_interfaces/action/PatrolAction "{start_patrol: true, patrol_type: random, behavior_tree:}"
```
