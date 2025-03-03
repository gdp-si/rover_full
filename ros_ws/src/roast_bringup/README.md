# Roast Bringup

This is the entry point for the robot (at bootup). It contains the launch files for the robot.

## Launch Files

- [`robot_bringup.launch.py`](launch/bringup.launch.py) (contains the launch files for the robot)

To run the launch file, run the following command:

```bash
# Without namespace (more suitable for the single robot)
ROBOT_TYPE=8-inch # or 10-inch
ros2 launch roast_bringup robot_bringup.launch.py

# With namespace (more suitable for the multirobot)
ROBOT_TYPE=8-inch # or 10-inch
ros2 launch roast_bringup robot_bringup.launch.py namespace:=<namespace>
```

where `<namespace>` is the namespace of the robot.
