//put robot-specific names here:
string g_urdf_base_frame_name("base_link");
string g_urdf_flange_frame_name("flange_frame");
string g_joint_states_topic_name("/irb140/joint_states");
string g_traj_pub_topic_name("/irb140/arm_controller/command");
//string g_traj_pub_topic_name("joint_path_command");
string g_traj_as_name("dummy"); //dummy--not used for irb120
std::vector<std::string> g_jnt_names{"joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
bool g_use_trajectory_action_server = false; //do NOT use action server; use traj download instead

