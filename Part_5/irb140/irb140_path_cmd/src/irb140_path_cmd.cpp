#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>


int main(int argc, char** argv){
	
	ros::init(argc, argv, "ABB_140_goal_sender");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<trajectory_msgs::JointTrajectory>("joint_path_command",1);

        trajectory_msgs::JointTrajectory traj;
	trajectory_msgs::JointTrajectoryPoint point0;
	trajectory_msgs::JointTrajectoryPoint point1;
	trajectory_msgs::JointTrajectoryPoint point2;
	trajectory_msgs::JointTrajectoryPoint point3;
	trajectory_msgs::JointTrajectoryPoint point4;


	traj.joint_names.push_back("joint_1");
	traj.joint_names.push_back("joint_2");
	traj.joint_names.push_back("joint_3");
	traj.joint_names.push_back("joint_4");
	traj.joint_names.push_back("joint_5");
	traj.joint_names.push_back("joint_6");

	point0.positions.clear();
	point1.positions.clear();
	point2.positions.clear();
	point3.positions.clear();
	point4.positions.clear();

	for(int i=0; i < 6; i++){
	  if(i == 5){
	    point0.positions.push_back(1.0);
	    point1.positions.push_back(2.0);
	    point2.positions.push_back(3.0);
	    point3.positions.push_back(4.0);
	    point4.positions.push_back(5.0);
	  }
	  else{
 	    point0.positions.push_back(0.0);
	    point1.positions.push_back(0.0);
	    point2.positions.push_back(0.0);
	    point3.positions.push_back(0.0);
	    point4.positions.push_back(0.0);
	  }
	}

	point0.time_from_start = ros::Duration(4.0);
	point1.time_from_start = ros::Duration(8.0);
	point2.time_from_start = ros::Duration(12.0);
	point3.time_from_start = ros::Duration(16.0);
	point4.time_from_start = ros::Duration(20.0);
	
	traj.points.clear();
	traj.header.stamp = ros::Time::now();
	traj.points.push_back(point0);
	traj.points.push_back(point1);
	traj.points.push_back(point2);
	traj.points.push_back(point3);
	traj.points.push_back(point4);

	for(int i = 0; i < 3; i++){
	  ROS_INFO("Attempting to publish trajectory to ABB 140...");
	  pub.publish(traj);
	  ros::spinOnce();
	  ros::Duration(5.0).sleep();
	}

	return 0;
}



