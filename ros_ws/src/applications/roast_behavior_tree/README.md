## Roast Behavior Tree

This package contains a behavior tree for the Roast robot. The behavior tree is implemented using the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

### Dependencies

 - [behaviortree_cpp_v3](https://github.com/BehaviorTree/BehaviorTree.CPP.git) Should be installed to use the behavior tree. Version 3.8 is used in the package
 - [Groot](https://github.com/BehaviorTree/Groot) Behavior tree editor. Used to create and edit the behavior tree. Version 3.8 is used in the package


### Available plugins

#### Actions

 - `SendCommandCenterMessage` : Sends a message to the command center
 - `MoveForward` : Moves the robot forward
 - `MoveBackward` : Moves the robot backward
 - `Patrol` : Moves the robot in a patrolling pattern
 - `ThreatTracking` : Tracks the threat

### Conditions

 - `IsThreatDetected` : Checks if a threat is detected
 - `RobotAtHome` : Checks if the robot is at home
 - `BatteryLow` : Checks if the battery is low

### Usage

To run the behavior tree, run the following command:

```bash
    ros2 launch roast_behavior_tree roast_behavior_tree.launch.py
```

### Description

This package contains the behavior tree core engine that handles the robot's behavior. To develop more custom plugins for the behavior tree, please refer to the [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) or check the existing plugins in the package.

 - The package provides a `BTActionNode` and `BTServiceNode` that can be wrapped around the plugin classes to make them compatible with the behavior tree engine.
 - Once the plugins are defined, we should be telling the behavior tree engine about the plugins. This is done by registering the plugins in the `register_nodes` function in the `roast_behavior_tree.cpp` file.
 - Then, in `CMakelists.txt`, we should add the plugin files to the `add_library` function.
 - Finally, we should tell the behavior tree engine that the plugins are available by adding the plugin name in the `launch` description in the `roast_behavior_tree.launch.py` file.
