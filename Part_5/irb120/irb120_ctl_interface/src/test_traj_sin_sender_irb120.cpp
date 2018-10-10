//test_traj_sin_sender_irb120.cpp:
//wsn, Oct 2018
//a simple node to illustrate how to publish a trajectory to topic "joint_path_command"
// intended to work with example_robot_interface node, 
// but should work as well with real ROS-Industrial "motion_download_interface" node--


//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_traj_sin_sender"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    
    //ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands
    trajectory_msgs::JointTrajectory new_trajectory; //   trajectory message
    trajectory_msgs::JointTrajectoryPoint trajectory_point0; //points to populate the trajectory message
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2,trajectory_point3,trajectory_point4,trajectory_point5,trajectory_point6; 
    
    new_trajectory.points.clear(); //fill in all the joint names
    new_trajectory.joint_names.push_back("joint1");
    new_trajectory.joint_names.push_back("joint2");
    new_trajectory.joint_names.push_back("joint3");
    new_trajectory.joint_names.push_back("joint4");
    new_trajectory.joint_names.push_back("joint5");
    new_trajectory.joint_names.push_back("joint6");    

    ros::Rate sleep_timer(1.0); //1Hz update rate
    

    // build an example trajectory:
    trajectory_point0.positions.clear();
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();    
    
    //specify start pose (all zeros) and two points:
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point0.positions.push_back(0.0); //all zeros
        trajectory_point1.positions.push_back(0.0); // 

    }
    //specify arrival time (from start of trajectory)
    trajectory_point0.time_from_start = ros::Duration(1.0); //1 second to arrive
    
    //put this point into trajectory message
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    new_trajectory.points.push_back(trajectory_point0); //contains a single point--home
 
    ROS_INFO("sending robot to home position: ");
    int npts = new_trajectory.points.size();
    ROS_INFO("num pts in initial traj = %d",npts);
    for (int i=0;i<3;i++) //send this twice, just to make sure have communication w/ subscriber
    {
        pub.publish(new_trajectory);
            ros::spinOnce();
            ros::Duration(1.0).sleep();
    }
    double amp1 = 1; //some amplitudes
    double amp2 = 0.3;
    double amp3 = 0.5;
    double amp4 = 0.4;
    double amp5 = 0.5;
    double amp6 = 0.6;
    double omega = 1.0; //pretty slow
    double dt = 0.1; //time step for trajj
    double Tfinal = 20.0; //run for this long
    double t=0;
    int n_traj_pts = Tfinal/dt;
    
    new_trajectory.points.clear();
    for (int ipt =0;ipt<n_traj_pts;ipt++) {
             trajectory_point1.positions[0] = amp1*sin(omega*t);
             trajectory_point1.positions[1] = amp2*sin(omega*t);
             trajectory_point1.positions[2] = amp3*sin(omega*t);
             trajectory_point1.positions[3] = amp4*sin(omega*t);
             trajectory_point1.positions[4] = amp5*sin(omega*t);
             trajectory_point1.positions[5] = amp6*sin(omega*t);
             trajectory_point1.time_from_start = ros::Duration(t); 
             t+=dt;
                 
             new_trajectory.points.push_back(trajectory_point1); // add this trajectory point to the trajectory message
        
    }
    new_trajectory.header.stamp = ros::Time::now();


     npts = new_trajectory.points.size(); 
     int njnts = new_trajectory.points[0].positions.size();
    ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);
    pub.publish(new_trajectory);
    //don't die yet--make sure message gets received
     for (int i=0;i<3;i++) {
            sleep_timer.sleep();
     }

    return 0;
} 

