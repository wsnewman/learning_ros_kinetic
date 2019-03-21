# irb120_control
loads /irb140_traj_control.yaml
starts joint_state_controller
starts robot_state_publisher on topic /irb140/joint_states

Launch file loads controllers into Gazebo and starts "robot_state_publisher".  The launch file irb140_control.launch is included in irb140_description/irb140.launch
    
