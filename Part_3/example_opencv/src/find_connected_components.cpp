//test use of blob detection
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

#include <iostream>

using namespace cv;
using namespace std;

//hard-coded image sizes for testing
Mat_<uchar> C(3,3);
Mat_<int>labelImage(3,3);


void blob_color(void) {
    
    //find the regions:  labelImage will contain integers corresponding to blob labels
    int nLabels = connectedComponents(C, labelImage, 4); //4 vs 8 connected regions
    //print out the region labels:
    cout << "labelImage = " << endl << " " << labelImage << endl << endl;
    
    //colorize the regions and display them:
    std::vector<Vec3b> colors(nLabels);
    colors[0] = Vec3b(0, 0, 0);//background
    //assign random color to each region label
    for(int label = 1; label < nLabels; ++label){
        colors[label] = Vec3b( (rand()&255), (rand()&255), (rand()&255) );
    }
    
    Mat dst(C.size(), CV_8UC3);  //create an image the same size as input image
    //for display image, assign colors to regions
    for(int r = 0; r < dst.rows; ++r){
        for(int c = 0; c < dst.cols; ++c){
            int label = labelImage.at<int>(r, c);
            Vec3b &pixel = dst.at<Vec3b>(r, c);
            pixel = colors[label];
         }
     }
    //display the result
    imshow( "Connected Components", dst );
}



int main(int argc, char** argv) {
    ros::init(argc, argv, "blob_finder");
    ros::NodeHandle n; //        
    
    //create bw image:
    C(0,0)=0;
    C(0,1)=0;
    C(0,2)=255;
    C(1,0)=0;
    C(1,1)=0;
    C(1,2)=0;
    C(2,0)=0;
    C(2,1)=255;
    C(2,2)=255;

    cout << "C = " << endl << " " << C << endl << endl;
    blob_color(); //find connected components and print and display them
    
    namedWindow( "Image", WINDOW_AUTOSIZE);
    namedWindow( "Connected Components", WINDOW_AUTOSIZE);    
    imshow( "Image", C ); //display it
    waitKey(0);

    return 0;
}
