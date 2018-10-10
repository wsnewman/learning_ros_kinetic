# magic_object_finder
This package contains a stand-in for a perception-based object finder.
The "magic"  object finder merely subscribes to  the Gazebo topic /gazebo/model_states, and
it looks for a name match between named gazebo models and a model name specified by an action client.
If there is no match, the result message contains a code for object-not-found.  If there is a name match,
the result message will contain the pose of the named object.

The intent of this node is to act as an equivalent object finder based on sensory processing.
It should be replaced by an equivalent action server that performs sensory interpretation.

## Example usage
Start up a gazebo simulation,  e.g.:
`roslaunch irb120_description irb120.launch`

Start the magic object finder action server:
`rosrun magic_object_finder magic_object_finder`

Query the magic object finder for the pose of a named part.  E.g., insert an ariac "gear_part" into
gazebo, and run the example client:
`rosrun magic_object_finder magic_object_finder_action_client`

You should observe that the model coordinates reported by Gazebo in the left panel of the gui are
identical to the model coordinates received by the action client via  the magic object finder action server.


    
