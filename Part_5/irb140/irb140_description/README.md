# irb140_description

This model was created by wsn, November, 2018, based on:
https://github.com/FreddyMartinez/abb_irb140_support

Dynamic properties (inertias, transmissions, torque limits, ...) are not to be trusted.
Robot constraints, including speed limits, torque limits and joint range limits, should be
checked.

At present, this model does not include a collision model for the joints (which could be
added, but has not.  Possibly will require trimming the collision models to avoid
interference issues in Gazebo).  Only the tool has a collision model.

The tool also has a "sticky fingers" plug-in to enable emulation of a vacuum gripper.


The irb140 model is launched as: 
`roslaunch irb140_description irb140.launch`

example action client is:
`rosrun cartesian_motion_commander example_block_grabber`

    
