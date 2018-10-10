# irb120_task_commander
Example task-level program for irb120 robot.  Specifies goals in Cartesian space and invokes
Cartesian moves.  Uses generic cartesian planner.

## Example usage
Start up simulation of irb120 robot:
`roslaunch irb120_description irb120.launch`
This launch file also starts up the control interface, which subscribes to the topic "joint_path_command"
This node is now ready to receive and execute trajectory messages published to "joint_path_command".

Run an example task commmander:
`rosrun irb120_task_commander.cpp irb120_task_commander.cpp`

Note: this motion can push a part.  E.g., insert a model gear_part_ariac in Gazebo and set its
coordinates to (0.3,0.15,0), then run the above task commander to push the part.

### Reactive task commander
This version has an action client that inquires re/ the pose of the part of interest (ARIAC gear part)
and computes a sequence of trajectories to touch the top of the part then withdraw.  Requires
that the "magic" object finder is running.

Start up simulation of irb120 robot:
`roslaunch irb120_description irb120.launch`

Start up the magic object finder, which gets model pose information directly from Gazebo
`rosrun magic_object_finder magic_object_finder`

Run the reactive task commmander:
`rosrun irb120_task_commander irb120_reactive_task_commander`
    
