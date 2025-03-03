# Roast Interfaces

This package contains the interfaces for the robot. Interfaces are the messages, services, and actions that are used to communicate between the nodes. _This package is used by the other packages in the workspace._

## Messages

- [Diagnostics.msg](msg/Diagnostics.msg) (contains the diagnostics information for the robot)
- [Robot.msg](msg/Robot.msg) (contains the robot information for the robot)
- [Vision.msg](msg/Vision.msg) (contains the vision information for the robot)
- [ObjectTracking.msg](msg/ai/ObjectTracking.msg) (contains the object tracking information for the robot)

## Services

- [Fan.srv](srv/Fan.srv) (contains the fan service for the robot)
- [Led.srv](srv/Led.srv) (contains the led service for the robot)
- [Jetson.srv](srv/Jetson.srv) (contains the jetson service for the robot)
- [NVPModel.srv](srv/NVPModel.srv) (contains the nvp model service for the robot)

### Hardware IO

- [SprayModule.srv](srv/hardware_io/SprayModule.srv) (contains the spray module service for the robot)

## Actions

- [ConnectToChargingStation.action](action/ConnectToChargingStation.action) (contains the connect to charging station action for the robot)
- [GoToPose.action](action/GoToPose.action) (contains the go to pose action for the robot)
- [TrackTarget.action](action/TrackTarget.action) (contains the track target action for the robot)
- [SendCommandCenterMessage.action](action/SendCommandCenterMessage.action) (contains the send command center message action for the robot)
