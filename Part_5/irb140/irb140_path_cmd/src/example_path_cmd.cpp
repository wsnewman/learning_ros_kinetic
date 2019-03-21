//example_path_cmd.cpp
//wsn, 11/18

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
#include <sensor_msgs/JointState.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>


using namespace std;
#define VECTOR_DIM 6 // e.g., a 6-dof vector
const double dt_traj = 0.02; // time step for trajectory interpolation
int g_done_count = 0;
int g_done_move = true;
Eigen::VectorXd g_q_vec_arm_Xd;
vector<int> g_arm_joint_indices;
vector<string> g_jnt_names;
//TEST: deliberately limit joint velocities to very small values
double g_qdot_max_vec[] = {1, 1, 1, 1, 1, 1}; //put real vel limits here
int ans;

double g_home_pose[] = {0,0,0,0,0,0};
double g_mantis_pose[] = {0,0,0,0,1.57,0};


void map_arm_joint_indices(vector<string> joint_names) {
    //vector<string> joint_names = joint_state->name;
    //   vector<string> jnt_names;

    g_arm_joint_indices.clear();
    int index;
    int n_jnts = VECTOR_DIM;
    //cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    for (int j = 0; j < VECTOR_DIM; j++) {
        j_name = g_jnt_names[j]; //known name, in preferred order
        for (int i = 0; i < n_jnts; i++) {
            if (j_name.compare(joint_names[i]) == 0) {
                index = i;
                //cout<<"found match at index = "<<i<<endl;
                g_arm_joint_indices.push_back(index);
                break;
            }
        }
    }
    cout << "indices of arm joints: " << endl;
    for (int i = 0; i < VECTOR_DIM; i++) {
        cout << g_arm_joint_indices[i] << ", ";
    }
    cout << endl;
}

void set_jnt_names() {
    g_jnt_names.push_back("joint_1");
    g_jnt_names.push_back("joint_2");
    g_jnt_names.push_back("joint_3");
    g_jnt_names.push_back("joint_4");
    g_jnt_names.push_back("joint_5");
    g_jnt_names.push_back("joint_6");
}

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    //joint_states_ = js_msg; // does joint-name mapping only once
    if (g_arm_joint_indices.size() < 1) {
        int njnts = js_msg.position.size();
        ROS_INFO("finding joint mappings for %d jnts", njnts);
        map_arm_joint_indices(js_msg.name);
    }
        for (int i = 0; i < VECTOR_DIM; i++) {
            g_q_vec_arm_Xd[i] = js_msg.position[g_arm_joint_indices[i]];
        }
        cout << "CB: q_vec_arm: " << g_q_vec_arm_Xd.transpose() << endl;
}

//need to reference realistic joint velocity limits to compute min transition times
double transition_time(Eigen::VectorXd dqvec) {
    double t_max = fabs(dqvec[0]) / g_qdot_max_vec[0];
    //cout<<"qdot max: "<<qdot_max_vec_.transpose()<<endl;
    double ti;
    for (int i = 1; i < VECTOR_DIM; i++) {
        ti = fabs(dqvec[i]) / g_qdot_max_vec[i];
        if (ti > t_max) t_max = ti;
    }
    return t_max;
}

//given a path, qvecs, comprised of a sequence of 6DOF poses, construct
// a corresponding trajectory message w/ plausible arrival times
// re-use joint naming, as set by set_jnt_names
//arrival time follows from running joints at max feasible velocities...
// but init vel limits to low values;
// need to upgrade this to accept an arrival time
void stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 

    trajectory_point1.positions.clear();

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear();
    for (int i = 0; i < VECTOR_DIM; i++) {
        new_trajectory.joint_names.push_back(g_jnt_names[i].c_str());
    }

    new_trajectory.header.stamp = ros::Time::now();  
    Eigen::VectorXd q_start, q_end, dqvec, qdot_vec;
    double del_time;
    double net_time = 0.0;
    q_start = qvecs[0];
    q_end = qvecs[0];
    qdot_vec.resize(VECTOR_DIM);
    qdot_vec<<0,0,0,0,0,0;
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 
    ROS_INFO("stuffing trajectory");
    //trajectory_point1.positions = qvecs[0];

    trajectory_point1.time_from_start = ros::Duration(net_time);
    for (int i = 0; i < VECTOR_DIM; i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
        trajectory_point1.velocities.push_back(0.0);
    }
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    //add the rest of the points from qvecs


    for (int iq = 1; iq < qvecs.size(); iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end - q_start;
        //cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time < dt_traj)
            del_time = dt_traj;
        //cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time += del_time;
        //ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i = 0; i < VECTOR_DIM; i++) { //copy over the joint-command values
            trajectory_point1.positions[i] = q_end[i];
            trajectory_point1.velocities[i] = dqvec[i]/del_time;
        }
        //trajectory_point1.positions = q_end;
        trajectory_point1.time_from_start = ros::Duration(net_time);
        new_trajectory.points.push_back(trajectory_point1);
    }
  //display trajectory:
    for (int iq = 1; iq < qvecs.size(); iq++) {
        cout<<"traj pt: ";
                for (int j=0;j<VECTOR_DIM;j++) {
                    cout<<new_trajectory.points[iq].positions[j]<<", ";
                }
        cout<<endl;
        cout<<"arrival time: "<<new_trajectory.points[iq].time_from_start.toSec()<<endl;
    }
}

ros::Publisher pub; 
int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "example_path_move"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    //this is the correct topic and message type for Motoman streaming
    
    
    cout<<"enter 1 for simu, 2 for ROS-I: ";
    cin>>ans;
    if (ans==1) {
        ROS_INFO("will publish to /irb140/arm_controller/command");
        ros::Publisher pub_simu = nh.advertise<trajectory_msgs::JointTrajectory>("/irb140/arm_controller/command", 1);
        pub = pub_simu;
    }
    else if (ans==2) {
        //this is the correct topic and message type for Motoman streaming
        ros::Publisher pub_rosi = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command", 1);
        ROS_INFO("will publish to /joint_path_command");
        pub = pub_rosi;
        }    
    // Eigen::VectorXd q_pre_pose;
    Eigen::VectorXd q_vec_arm_pose;
    g_q_vec_arm_Xd.resize(VECTOR_DIM);
    q_vec_arm_pose.resize(VECTOR_DIM);
    std::vector<Eigen::VectorXd> des_path;
    trajectory_msgs::JointTrajectory des_trajectory; // empty trajectory   
    set_jnt_names(); //fill a vector of joint names in DH order, from base to tip

    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 1, jointStatesCb);

    // warm up the joint-state callbacks; want to make sure the joint states are valid
    cout << "warming up callbacks..." << endl;
    while (g_arm_joint_indices.size() < 1) {
        ros::spinOnce();
        ros::Duration(1).sleep();
        ROS_INFO("waiting for joint states...");
    }

    //get current pose of arm:  
    cout << "current pose:" << g_q_vec_arm_Xd.transpose() << endl;
    double init_pose_err = fabs(g_q_vec_arm_Xd[0]-g_home_pose[0])+fabs(g_q_vec_arm_Xd[1]-g_home_pose[1])+fabs(g_q_vec_arm_Xd[2]-g_home_pose[2]);
    ROS_INFO("initial pose err for first three joints = %f",init_pose_err);
    if (init_pose_err>0.1) {
        ROS_WARN("ROBOT IS NOT CLOSE ENOUGH TO HOME POSE; GIVING UP");
        return 0;
    }
    
    //init first command to current joint state values, to warm up streamer:
    ROS_WARN("warming up trajectory publications...");
    ROS_WARN("robot should not move!");
    
    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd); //start from current pose  
    des_path.push_back(g_q_vec_arm_Xd); //command to go to current pose            
                
    stuff_trajectory(des_path, des_trajectory); //convert path to traj    
    for (int i=0;i<3;i++) {
            des_trajectory.header.stamp = ros::Time::now(); //update time stamp to avoid rejection        
            pub.publish(des_trajectory);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
                }    
    
    //now assemble a trajectory to fold the robot safely:
    des_path.clear();
    des_path.push_back(g_q_vec_arm_Xd); //start from current pose  
    
    for (int i=0;i<VECTOR_DIM;i++) {
        q_vec_arm_pose[i] = g_mantis_pose[i];
    }
    des_path.push_back(q_vec_arm_pose); 
    
    stuff_trajectory(des_path, des_trajectory); //convert path to traj    
    des_trajectory.header.stamp = ros::Time::now(); //update time stamp to avoid rejection        
    pub.publish(des_trajectory);
    ros::spinOnce();
    ros::Duration(2).sleep();
    
    ros::spinOnce(); //get the new joint angles
    ROS_INFO_STREAM("angles at: "<<endl<<g_q_vec_arm_Xd.transpose()<<endl);
 

    return 0;
}
    
    

