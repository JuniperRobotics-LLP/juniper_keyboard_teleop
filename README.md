# README.MD

This node utilizes the SDL library to track keyboard inputs and publish a ROS2 geometry_msgs/twist. If you are running through ssh, make sure you pass in the correct flags when logging in to allow access to the window pop for SDL. 

```
ssh -X -t juniper@IP-Address
```

To start the node 

```
ros2 run juniper_keyboard_teleop juniper_keyboard_teleop
```

## Topics
Node publishes a geometry_msgs/twist message on the default topic of `\cmd_vel`.

## Parameters
There are two parameters: the linear speed and the angular speed. The defaults are listed below:
- `linear_vel` = 0.35
- `angular_vel` = 2.5

They can be changed when starting the node:
```
ros2 run juniper_keyboard_teleop juniper_keyboard_teleop --ros-args -p linear_vel:=0.5 -p angular_vel:=3.0
```

Or they can also be changed dynamically once the node is running

```
ros2 param set /juniper_keyboard_teleop linear_vel 0.75
```
```
ros2 param set /juniper_keyboard_teleop angular_vel 3.75
```

## Testing with turtlesim
Open a terminal and start turtlesim

```
ros2 run turtlesim turtlesim_node
```

In a separate terminal, start this node remapping the topic to be `/turtle1/cmd_vel` and increasing the speed. 
```
ros2 run juniper_keyboard_teleop juniper_keyboard_teleop --ros-args -p linear_vel:=7.0 -p angular_vel:=10.0 -r /cmd_vel:=/turtle1/cmd_vel
```