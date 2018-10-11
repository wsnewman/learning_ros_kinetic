// action server to respond to perception requests
//this one is "magic", because it gets model states from Gazebo
// more realistically, model states would need to be obtained from sensor interpretation
// Wyatt Newman, Oct 2018

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<magic_object_finder/magicObjectFinderAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <xform_utils/xform_utils.h> //not needed yet
using namespace std;
gazebo_msgs::ModelStates g_ms_msg;

bool g_got_new_state = false;
void modelStatesCb(const gazebo_msgs::ModelStates& ms_msg) {
    int n_objects = ms_msg.name.size();
    //ROS_INFO("there are %d objects",n_objects);
    g_ms_msg=ms_msg;
    g_got_new_state=true;
}

class MagicObjectFinder {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<magic_object_finder::magicObjectFinderAction> object_finder_as_;

    // here are some message types to communicate with our client(s)
    magic_object_finder::magicObjectFinderGoal goal_; // goal message, received from client
    magic_object_finder::magicObjectFinderResult result_; // put results here, to be sent back to the client when done w/ goal
    magic_object_finder::magicObjectFinderFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    ros::Publisher pose_publisher_;// = nh.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);


    bool find_named_object(string  object_name, geometry_msgs::PoseStamped &object_pose);

public:
    MagicObjectFinder(); //define the body of the constructor outside of class definition

    ~MagicObjectFinder(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<magic_object_finder::magicObjectFinderAction>::GoalConstPtr& goal);
    //XformUtils xformUtils_;
};

MagicObjectFinder::MagicObjectFinder() :
object_finder_as_(nh_, "object_finder_action_service", boost::bind(&MagicObjectFinder::executeCB, this, _1), false) {
    ROS_INFO("in constructor of MagicObjectFinder...");
    // do any other desired initializations here...specific to your implementation
 
    object_finder_as_.start(); //start the server running

    pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("triad_display_pose", 1, true);
    //tfListener_ = new tf::TransformListener; //create a transform listener    
}

//look for named object
bool MagicObjectFinder::find_named_object(string  object_name, geometry_msgs::PoseStamped &object_pose) {
    
    bool found_object = false;
    g_got_new_state = false;
    while (!g_got_new_state) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    int n_objects = g_ms_msg.name.size();
    ROS_INFO("callback reports %d objects",n_objects);
    int object_num=0;
    //look for a name match:
    for (object_num=0;object_num<n_objects;object_num++) {
        if (object_name==string(g_ms_msg.name[object_num])) {
            ROS_INFO("found object as object number %d",object_num);
            break;
        }
    }
    if (object_num>=n_objects) {
        ROS_WARN("object not found");
        return false;
    }
    //if here, found (first instance of) object name on list
    //set corresponding pose values
    found_object=true;

    object_pose.header.frame_id = "world";
    object_pose.pose.position.x = g_ms_msg.pose[object_num].position.x;
    object_pose.pose.position.y = g_ms_msg.pose[object_num].position.y;
    object_pose.pose.position.z = g_ms_msg.pose[object_num].position.z;
    object_pose.pose.orientation.x = g_ms_msg.pose[object_num].orientation.x;
    object_pose.pose.orientation.y = g_ms_msg.pose[object_num].orientation.y;
    object_pose.pose.orientation.z = g_ms_msg.pose[object_num].orientation.z;
    object_pose.pose.orientation.w = g_ms_msg.pose[object_num].orientation.w;
    pose_publisher_.publish(object_pose);
    return found_object;

}


void MagicObjectFinder::executeCB(const actionlib::SimpleActionServer<magic_object_finder::magicObjectFinderAction>::GoalConstPtr& goal) {
    string object_name(goal->object_name);
    geometry_msgs::PoseStamped object_pose;

    bool found_object = find_named_object(object_name, object_pose);
    if (found_object) {
                ROS_INFO_STREAM("found object "<<object_name<<endl);
                result_.found_object_code = magic_object_finder::magicObjectFinderResult::OBJECT_FOUND;
                result_.object_pose = object_pose;
                object_finder_as_.setSucceeded(result_);
    } else {
                ROS_WARN("could not find requested object");
                result_.found_object_code = magic_object_finder::magicObjectFinderResult::OBJECT_NOT_FOUND;
                object_finder_as_.setAborted(result_);
    }
 

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "magic_object_finder_node"); // name this node 
    ros::NodeHandle nh;
    ROS_INFO("instantiating the object finder action server: ");

    MagicObjectFinder object_finder_as; // create an instance of the class "MagicObjectFinder"
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCb);

    ROS_INFO("going into spin");
    ros::spin();

    return 0;
}

