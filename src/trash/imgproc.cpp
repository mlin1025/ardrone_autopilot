#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "features.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

struct ImageProcessor
{
    bool cv_enabled = false;
    bool boxSended = false;
    ros::Publisher boxSendler;
    ros::Publisher imgPublisher;
    ros::Publisher circlePublisher;
    std::vector<double> cameraDistortion;
    boost::array<double, 9> cameraMatrix;
};

struct ImageProcessor imgProc;

static CirclesMessage cMessage;

// Convert target information to string
// Format:
// boxLeft,boxRight,boxTop,boxBottom circleX,circleY,circleWidth,circleHeight,circleInTheBox.
/*
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
         str.data += std::to_string(b) + ' ';
    }
}
*/

void cmsg2BoxmultiArray(CirclesMessage& cmsg,
                     std_msgs::Float32MultiArray& boxToSend) {
        std::vector<float> vec1 = {cmsg.box.left, cmsg.box.right, cmsg.box.top, cmsg.box.bottom};
        /*msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        boxToSend.layout.dim[0].size = vec1.size();
        boxToSend.layout.dim[0].stride = 1;
        boxToSend.layout.dim[0].label = "box";*/
        boxToSend.data.insert(boxToSend.data.end(), vec1.begin(), vec1.end());
}

void cmsg2multiArray(CirclesMessage& cmsg,
                        std_msgs::Float32MultiArray& msg) {
        std::vector<float> vec1;
        for (size_t i = 0; i != cmsg.inTheBox.size(); ++i) {
            vec1.clear();
            vec1 = {
                    cmsg.circles[i].center.x,
                    cmsg.circles[i].center.y,
                    cmsg.circles[i].size.width,
                    cmsg.circles[i].size.height,
                    cmsg.inTheBox[i]
            };
            /*msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[i].size = vec1.size();
            msg.layout.dim[i].stride = 1;
            msg.layout.dim[i].label = "circle" + std::to_string(i); */
            msg.data.insert(msg.data.end(), vec1.begin(), vec1.end());
        }
} 


// Get and process the image from camera

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
   if (imgProc.cv_enabled) {
        // Process image and get target information
        processImage(cv_ptr->image, cMessage);
        
        // Convert message and send it
        std_msgs::Float32MultiArray msg;
        std_msgs::Float32MultiArray sendBox;
        if (!imgProc.boxSended) {
            cmsg2BoxmultiArray(cMessage,  sendBox);
            imgProc.boxSendler.publish(sendBox);
            imgProc.boxSended = true;
        } else {
            cmsg2multiArray(cMessage, msg);
        }
        imgProc.circlePublisher.publish(msg);
    }

    // Send image
    imgProc.imgPublisher.publish(cv_ptr->toImageMsg());

}


void onCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    imgProc.cameraDistortion = cam_info->D;
    imgProc.cameraMatrix = cam_info->K;
}

// Enable/disable image processing

void onEnable(const std_msgs::Empty& toggle_msg) {
    imgProc.cv_enabled = !imgProc.cv_enabled;
    if (imgProc.cv_enabled) {
        std::cout << "Image processing enabled.\n";
    } else {
        std::cout << "Image processing disabled.\n";
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imgproc");
    ros::NodeHandle node;
    
    // M button
    ros::Subscriber enableSub = 
            node.subscribe("cv/enable", 1, onEnable);
    
    imgProc.imgPublisher = 
            node.advertise<sensor_msgs::Image>("/out/image", 5);

    imgProc.boxSendler = 
            node.advertise<std_msgs::Float32MultiArray>("box", 1);
    imgProc.circlePublisher = 
            node.advertise<std_msgs::Float32MultiArray>("target", 5);
    
    
    ros::Subscriber sub1 = 
            node.subscribe("/in/image", 5, onImage);
    ros::Subscriber sub2 = 
            node.subscribe("/ardrone/image_raw", 5, onImage);
    ros::Subscriber sub3 = 
            node.subscribe("/ardrone/camera_info", 5, onCameraInfo);

    ros::spin();
    //int count = 0;
    //++count;


    return 0;
}
