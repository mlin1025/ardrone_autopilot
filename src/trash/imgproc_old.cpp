#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "features.h"
#include <std_msgs/String.h>

struct ImageProcessor
{
    ros::Publisher imgPublisher;
    ros::Publisher circlePublisher;
    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};

struct ImageProcessor imgProc;

static CirclesMessage cMessage;

void cmsg2string (CirclesMessage& cmsg, std_msgs::String& str) {
    str.data += cmsg.box.left + ','
                + cmsg.box.right + ','
                + cmsg.box.top + ','
                + cmsg.box.bottom + ' ';
    for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
         float x = cmsg.circles[i].center.x;
         float y = cmsg.circles[i].center.y;
         float w = cmsg.circles[i].size.width;
         float h = cmsg.circles[i].size.height;
         bool b = cmsg.inTheBox[i];
         str.data += std::to_string(x) + ',';
         str.data += std::to_string(y) + ',';
         str.data += std::to_string(w) + ',';
         str.data += std::to_string(h) + ',';
         str.data += std::to_string(b);
         if (i != cmsg.inTheBox.size() - 1) str.data += ' ';
         else str.data += '.';
    }
}

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
        catch	 (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    processImage(cv_ptr->image, cMessage);

    imgProc.imgPublisher.publish(cv_ptr->toImageMsg());

    std_msgs::String cString;
    cmsg2string(cMessage, cString);
    imgProc.circlePublisher.publish(cString);
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
    imgProc.circlePublisher = node.advertise<std_msgs::String>("target", 5);
    ros::Subscriber sub1 = node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 = node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 = node.subscribe("/ardrone/camera_info", 5, onCameraInfo);
    

    ros::spin();
    int count = 0;
    ++count;


    return 0;
}
