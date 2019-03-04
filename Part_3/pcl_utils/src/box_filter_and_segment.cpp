//find_filter_and_segment.cpp
// try to segment objects.  Start w/ filtering

// voxel-grid filtering: pcl::VoxelGrid,  setInputCloud(), setLeafSize(), filter()
//wsn Feb 2019

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs


using namespace std;
extern PclUtils *g_pcl_utils_ptr; 

//this fnc is defined in a separate module, find_indices_of_plane_from_patch.cpp
extern void find_indices_of_plane_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, vector<int> &indices);

extern void find_indices_box_filtered_from_patch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
        pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr, Eigen::Vector3f box_pt_min, Eigen::Vector3f box_pt_max,
        vector<int> &indices);

extern Eigen::Affine3f compute_plane_affine_from_patch(pcl::PointCloud<pcl::PointXYZ>::Ptr patch_cloud_ptr); 

extern void find_indices_box_filtered(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,Eigen::Vector3f box_pt_min,  
 Eigen::Vector3f box_pt_max,vector<int> &indices);

// can't convert 
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >}’ 
//to ‘const PointCloudConstPtr& {aka const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&}’

double find_table_height(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,double z_min, double z_max, double dz) {
    vector<int> indices;
    pcl::PassThrough<pcl::PointXYZRGB> pass; //create a pass-through object    
    pass.setInputCloud(input_cloud_ptr); //set the cloud we want to operate on--pass via a pointer
    pass.setFilterFieldName("z"); // we will "filter" based on points that lie within some range of z-value
    double z_table = 0.0;
    int npts_max = 0;
    int npts;
    pcl::PointXYZRGB point;
    int ans;
    for (double z = z_min; z < z_max; z += dz) {
        pass.setFilterLimits(z, z + dz);
        pass.filter(indices); //  this will return the indices of the points in   transformed_cloud_ptr that pass our test
        npts = indices.size();
        /*
        if (npts>0) cout<<"num indices: "<<npts<<endl;
        //cout<<"enter 1: ";
        //cin>>ans;
        int i_index;
        for (int i=0;i<npts;i++) {
            i_index = indices[i];
            point = input_cloud_ptr->points[i_index];
            cout<<"point "<<i_index<<": x,y,z = "<<point.x<<", "<<point.y<<", "<<point.z<<endl;
        }
         * */
        if (npts>0) ROS_INFO("z=%f; npts = %d",z,npts);
        if (npts > npts_max) {
            npts_max = npts;
            z_table = z + 0.5 * dz;
        }
    }
    ROS_INFO("max pts %d at height z= %f",npts_max,z_table);
    return z_table;
}

Eigen::Affine3f compute_affine_cam_wrt_floor(double tilt_ang) {
     //pose of cam w/rt floor
    Eigen::Affine3f affine_cam_wrt_floor;
    Eigen::Matrix3f R_cam;
    Eigen::Vector3f O_cam;
    O_cam<<0,0,1.810;
    affine_cam_wrt_floor.translation() = O_cam;
    double cos_tilt = cos(tilt_ang);
    double sin_tilt = sin(tilt_ang);
    Eigen::Vector3f nvec,tvec,bvec;
    //[c 0 s
    // 0 1 0
    //-s 0 c]  or, try:
    //[s 0 c
    // 0 1 0
    // c 0 -s]
    // or rot about x-axis:
    //[1  0  0
    // 0  c -s
    // 0  s  c]     
    nvec<<1,0,0;
    tvec<<0,cos_tilt,sin_tilt;
    bvec<<0,-sin_tilt,cos_tilt;
    R_cam.col(0) = nvec;
    R_cam.col(1) = tvec;
    R_cam.col(2) = bvec;
    affine_cam_wrt_floor.linear() = R_cam;
    return affine_cam_wrt_floor;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_pts_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for pointcloud of planar points found
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_pts_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>); //ptr to selected pts from Rvis tool
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> indices;
    Eigen::Affine3f A_plane_wrt_camera;

    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    cout << "enter pcd file name: "; //prompt to enter file name
    cin >> fname;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *pclKinect_clr_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    pclKinect_clr_ptr->header.frame_id = "camera_depth_optical_frame";
    ROS_INFO("view frame camera_depth_optical_frame on topics pcd, planar_pts and downsampled_pcd");
    //<origin xyz="0.027 0.0 1.810" rpy="0.0 1.2 0"/>
    //define pose of the camera for transforming points:
    Eigen::Affine3f affine_cam_wrt_floor; //pose of cam w/rt floor

    
    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubPlane = nh.advertise<sensor_msgs::PointCloud2> ("planar_pts", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("downsampled_pcd", 1);
    ros::Publisher pubBoxFilt = nh.advertise<sensor_msgs::PointCloud2> ("box_filtered_pcd", 1);
    sensor_msgs::PointCloud2 ros_cloud, ros_planar_cloud, downsampled_cloud, ros_box_filtered_cloud; //here are ROS-compatible messages
    pcl::toROSMsg(*pclKinect_clr_ptr, ros_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(pclKinect_clr_ptr);

    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_kinect_ptr);
    cout << "done voxel filtering" << endl;

    cout << "num bytes in original cloud data = " << pclKinect_clr_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_kinect_ptr->points.size() << endl; // ->data.size()<<endl;    
    pcl::toROSMsg(*downsampled_kinect_ptr, downsampled_cloud); //convert to ros message for publication and display

    PclUtils pclUtils(&nh); //instantiate a PclUtils object--a local library w/ some handy fncs
    g_pcl_utils_ptr = &pclUtils; // make this object shared globally, so above fnc can use it too



    /*
    cout << " select a patch of points to find corresponding plane..." << endl; //prompt user action
    //loop to test for new selected-points inputs and compute and display corresponding planar fits 
    while(!pclUtils.got_selected_points()) {
        pubCloud.publish(ros_cloud);
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.3).sleep();
    }
    pclUtils.get_copy_selected_points(selected_pts_cloud_ptr); //get a copy of the selected points
    cout << "got new patch with number of selected pts = " << selected_pts_cloud_ptr->points.size() << endl;        
        
    A_plane_wrt_camera = compute_plane_affine_from_patch(selected_pts_cloud_ptr);  
     * */
    //transform cloud to plane coords:
    //A_floor_wrt_camera
    //pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera.inverse());
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_; private var of pcl_utils
    // vs: pclTransformed_ptr_(new PointCloud<pcl::PointXYZ>)
    
    double tilt_ang = -2.7709; //pi- 0.3707 = pi + 1.2 - pi/2 = pi/2 + 1.2
    //while (true) {
        //cout<<"enter tilt_ang: "<<endl;
        //cin>>tilt_ang;
        A_plane_wrt_camera = compute_affine_cam_wrt_floor( tilt_ang);
        cout<<"R_plane_wrt_cam: "<<endl<<A_plane_wrt_camera.linear()<<endl;

       //pcl::transformPointCloud(*pclKinect_clr_ptr, *transformed_cloud_ptr, A_plane_wrt_camera);
       pcl::transformPointCloud(*downsampled_kinect_ptr, *transformed_cloud_ptr, A_plane_wrt_camera);

    //pclTransformed_ptr_
    int npts = transformed_cloud_ptr->points.size();
    cout<<"npts transformed = "<<npts<<endl;
    /*
    Eigen::Vector3f pt;
    for (int i = 0; i < npts; ++i) {
        pt = transformed_cloud_ptr->points[i].getVector3fMap();
        cout<<pt.transpose()<<endl;
    }*/
    //    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // vs pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr
    
    //error: could not convert ‘transformed_cloud_ptr’ 
    //from ‘pcl::PointCloud<pcl::PointXYZRGB>::Ptr {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >}’ 
    //to ‘pcl::PointCloud<pcl::PointXYZ>::Ptr {aka boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >}’

    double table_height = find_table_height(transformed_cloud_ptr,-2.0, 2.0, 0.01);
    //}
    
    Eigen::Vector3f box_pt_min,box_pt_max;
    box_pt_min<<0.5,-0.5,0.02;
    box_pt_max<<2.0,0.5,0.5;
    int dim;

    
    while (ros::ok()) {
       cout<<"enter dim (0,1,2): ";
       cin>>dim;
       if ((dim>=0)&&(dim<3)) {
        cout<<"enter min val: ";
        cin>>box_pt_min(dim);
        cout<<"enter max val: ";
        cin>>box_pt_max(dim); }
       else { ROS_WARN("index out of range"); }

            //find pts coplanar w/ selected patch, using PCL methods in above-defined function
            //"indices" will get filled with indices of points that are approx co-planar with the selected patch
            // can extract indices from original cloud, or from voxel-filtered (down-sampled) cloud
            //find_indices_of_plane_from_patch(pclKinect_clr_ptr, selected_pts_cloud_ptr, indices);
            //find_indices_of_plane_from_patch(downsampled_kinect_ptr, selected_pts_cloud_ptr, indices);
            find_indices_box_filtered(transformed_cloud_ptr,box_pt_min,  box_pt_max,indices);
            pcl::copyPointCloud(*pclKinect_clr_ptr, indices, *box_filtered_cloud_ptr); //extract these pts into new cloud
            //the new cloud is a set of points from original cloud, coplanar with selected patch; display the result
            pcl::toROSMsg(*box_filtered_cloud_ptr, ros_box_filtered_cloud); //convert to ros message for publication and display
            pubBoxFilt.publish(ros_box_filtered_cloud);
            ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
            ros::Duration(0.3).sleep();
        }
        //pubCloud.publish(ros_cloud); // will not need to keep republishing if display setting is persistent
        //pubPlane.publish(ros_planar_cloud); // display the set of points computed to be coplanar w/ selection
        //pubDnSamp.publish(downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        //pubBoxFilt.publish(ros_box_filtered_cloud);
        //ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        //ros::Duration(0.3).sleep();


    return 0;
}
