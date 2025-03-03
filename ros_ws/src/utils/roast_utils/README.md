# Roast Utils

A collection of utilities for working with Roast.

## Utilities

- [`acceleration_estimator`](#acceleration-estimator) - A utility for estimating the acceleration of a robot (both linear and angular).
- [`motor_drive_calibrator`](#motor-drive-calibrator) - A utility for calibrating the motor drive of a robot.
- [`pid_tuner`](#pid-tuner) - A utility for tuning the PID parameters of a robot.
- [`odometry_tester`](#odometry-tester) - A utility for testing the odometry of a robot.
- [`patrol_points_builder`](#patrol-points-builder) - A utility for building patrol points for a robot.
- [`threat_pose_publisher`](#threat-pose-publisher) - A utility for publishing threat poses for a robot.
- [`rosbag2csv`](#rosbag2csv) - A utility for converting rosbag files to csv files.
- [`lifecycle_activator`](#lifecycle-activator) - A utility for activating the lifecycle of a node.

## Usage

### Acceleration Estimator

To evaluate the acceleration of the robot, run the following command:

```bash
ros2 run roast_utils acceleration_estimator --ros-args -p mode:=<mode> -p max_velocity_x:=<max_velocity_x> -p max_ang_velocity_z:=<max_ang_velocity_z>
```

where:

- `mode` is the mode of the acceleration estimator. It can be either `linear` or `angular` or `both`.
- `max_velocity_x` is the maximum linear velocity of the robot in the x-axis.
- `max_ang_velocity_z` is the maximum angular velocity of the robot in the z-axis.

### Motor Drive Calibrator

To calibrate the motor drive of the robot, run the following command:

```bash
ros2 run roast_utils motor_drive_calibrator --ros-args -p pulse_magnitude:=<pulse_magnitude> -p pulse_freq:=<pulse_freq> -p mode:=<mode>
```

where:

- `pulse_magnitude` is the magnitude of the pulse to be applied to the motor drive.
- `pulse_freq` is the frequency of the pulse to be applied to the motor drive.
- `mode` is the mode of the motor drive calibrator. It can be either `linear` or `angular`.

### PID Tuner

To tune the PID parameters of the robot, run the following command:

```bash
ros2 run roast_utils pid_tuner
```

### Odometry Tester

To test the odometry of the robot, run the following command:

```bash
ros2 run roast_utils odometry_tester --ros-args -p side_length:=<square_side_length>
```

where:

- `square_side_length` is the side length of the square to be traversed by the robot.

### Patrol Points Builder

To build patrol points for the robot, run the following command:

```bash
ros2 run roast_utils patrol_points_builder
```

| Note: The patrol points are saved in `RESOURCES_PATH` of package `roast_patrol` and may need to be moved to the `config` directory of the robot.

### Threat Pose Publisher

Stream the coordinates of the threat from a CSV file to the threat_pose topic.

- CSV file is of format: is_threat, y, x, threat_type, y_safe, x_safe
- ROS CSV file is of format: timestamp, timestamp_ns, frame, x, y, z, qx, qy, qz, qw

Usage:

```bash

# To stream ros messages from a csv file
ros2 run roast_ai threat_pose_publisher_csv -p type:=ros -p file_path:=<path_to_csv_file>

# To stream python data format from a csv file
ros2 run roast_ai threat_pose_publisher_csv -p type:=py -p file_path:=<path_to_csv_file>
```

where `type` is the type of data to be streamed and `file_path` is the path to the CSV file.

### rosbag2csv

Converts a rosbag file to a csv file.

 - Uses `sqlite3` as storage identifier.
 - Allows converting mutliple files in a single command.
 - Allows converting all topics in a single command.

Usage:

```bash
ros2 run roast_utils rosbag2csv
```

### Lifecycle Activator

Activates and deactivates the lifecycle of a node.

Usage:

```bash
# To activate the lifecycle of a node
ros2 run roast_utils lifecycle_activator <node_name> --activate

# To deactivate the lifecycle of a node
ros2 run roast_utils lifecycle_activator <node_name> --deactivate
```

where:
    node_name is the name of the node whose lifecycle is to be activated or deactivated.
