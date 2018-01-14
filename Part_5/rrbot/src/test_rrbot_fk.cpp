//test_rrbot_fk: tests rrbot_fk_ik library
// subscribes to joint values states;
// computes FK
// uses FK to compute IK
// compare IK solutions to actual answer

#include <rrbot/rrbot_kinematics.h> 
#include <sensor_msgs/JointState.h>

Eigen::VectorXd g_q_vec;
using namespace std;

void jointStatesCb(const sensor_msgs::JointState& js_msg) {
    for (int i = 0; i < NJNTS; i++) {
        g_q_vec[i] = js_msg.position[i];
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rrbot_fk_test");

    ros::NodeHandle nh;
    Eigen::Vector2d q_init;
    q_init<<0,0;
    g_q_vec= q_init; // init global q_vec

    ros::Subscriber joint_state_sub = nh.subscribe("rrbot/joint_states", 1, jointStatesCb);
    //warm up joint states:
    g_q_vec[0]=100;
    while (g_q_vec[0]>99) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    Eigen::Affine3d affine_flange;

    Rrbot_fwd_solver rrbot_fwd_solver;
    Rrbot_IK_solver rrbot_ik_solver;    
    Eigen::Vector3d flange_origin_wrt_world;
    int n_solns;
    //bool valid_q_elbow = false;
    //std::vector<double> q_elbow_solns;
    //q_elbow_solns.clear();
    std::vector<Eigen::Vector2d> q_solns;
    //double q_elbow, q_shoulder; 
    while (ros::ok()) {
        cout<<endl<<endl;
        ROS_INFO("angs: %f, %f", g_q_vec[0], g_q_vec[1]);

        affine_flange = rrbot_fwd_solver.fwd_kin_flange_wrt_world_solve(g_q_vec);
        //for (int i = 0; i < NJNTS; i++) {
        //    cout << "frame " << i << " w/rt world: " << endl;
        //    cout << rrbot_fwd_solver.get_frame(i) << endl;
        //}

        flange_origin_wrt_world = affine_flange.translation();
        cout << "FK: flange origin: " << flange_origin_wrt_world.transpose() << endl;
        cout << "R_flange = " << endl;
        cout << affine_flange.linear() << endl;
        Eigen::Quaterniond quat(affine_flange.linear());
        cout<<"equiv quat: "<<quat.x()<<", "<< quat.y()<<", "<<quat.z()<<", "<<quat.w()<<endl;

        //cout<<"robot pose: (q_shoulder,q_elbow) = ("<<g_q_vec[0]<<", "<<g_q_vec[1]<<")"<<endl;
        ROS_INFO("computing inverse kinematics: ");
        n_solns= rrbot_ik_solver.ik_solve(affine_flange, q_solns);      
       
        if (n_solns<1) cout << "found no viable IK solns" << endl;

        else {
            for (int i_soln=0;i_soln<n_solns;i_soln++) {
                cout<<"IK soln "<<i_soln<<": "<<q_solns[i_soln].transpose()<<endl;
            }
        }
        
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
}
