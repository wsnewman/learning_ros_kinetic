# irb140_path_cmd

demo program to move irb140, either gazebo sim or physical ROS-I interface

## Example usage
start up gazebo sim of robot (or start  up physical robot and ROS-I  bridge)
rosrun irb140_path_cmd interactive_path_cmd

will prompt for simu vs ROS-I (which have different command topic names)
will prompt for joint number and joint value,  package these  in a trajectory message and
publish the message to the command topic

## Running tests/demos
    
