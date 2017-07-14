#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "features.h"

struct ImageProcessor
{
    ros::Publisher imgPublisher;
    ros::Publisher vectorPublisher;
    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};

struct ImageProcessor imgProc;


void onImage(const sensor_msgs::Image::ConstPtr& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    processImage(cv_ptr->image);
    imgProc.imgPublisher.publish(cv_ptr->toImageMsg());
}


void onCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    imgProc.cameraDistortion = cam_info->D;
    imgProc.cameraMatrix = cam_info->K;
}


int main(int argc, char **argv)
{
    /* 
     * Vasya was here
     */

    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;
    imgProc.imgPublisher = node.advertise<sensor_msgs::Image>("/out/image", 5);
    ros::Subscriber sub1 = node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 = node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 = node.subscribe("/ardrone/camera_info", 5, onCameraInfo);
    

    ros::spin();
    int count = 0;
    ++count;


    return 0;
}
