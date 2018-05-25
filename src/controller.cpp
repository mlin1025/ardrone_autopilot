
/*  
    controller.cpp
  ----------------------------------------------------------------------------
  | Receives the image from the drone and sends it to the compVision.cpp,    |
  | that processes it and extracts information about the circles and the box.|
  |                                                                          |
  | Then receives processed information and sends it to the controller.      |
  | Some utility classes and functions can be found in the controlHelper.h.  |
  ----------------------------------------------------------------------------

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "controlHelper.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>



// Global control object to have an easy access to some parameters.
geometry_msgs::PoseStamped currentPose;
struct ControlCenter control;
bool optitrack = false;


// Enable/disable controller by button 'N'.

void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n"; 
}

// Recieves information and runs the control function the drone.
// Runs when the target information has been received.

void onControl() {
    if (control.enabled) {
        if(!optitrack){
            std::cout<<"No Optitrack Data!!! Check Connection";
        }else{
            geometry_msgs::Twist message;
            // Run the controller.
            message = control.pid.caculate(currentPose);
            // Send the message.
            control.cmdPublisher.publish(message);
        }
	} else {
	// Reset saved information if the controller was turned off.
        control.pid.reset();
    }
}
        
// Changing PID coefficients by buttons.

// Reducing.

void onPidD(const std_msgs::String& str) {
    // switch (str.data[0]) {
    //     case 'p': control.pid.kP += 0.005; // 'F1'
    //               break;
    //     case 'i': control.pid.kI += 0.001; // 'F3'
    //               break;
    //     case 'd': control.pid.kD += 0.005; // 'F5'
    //               break;
    // }
}

// Increasing.

void onPidI(const std_msgs::String& str) {
    // switch (str.data[0]) {
    //     case 'p': control.pid.kP -= 0.0005; // 'F2'
    //               break;
    //     case 'i': control.pid.kI -= 0.0005; // 'F4'
    //               break;
    //     case 'd': control.pid.kD -= 0.0005; // 'F6'
    //               break;
    // }
}

void onTruth(const geometry_msgs::PoseStamped& msg) {
    currentPose = msg;
    optitrack=true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle node;

    ros::Subscriber enableSub = 
            node.subscribe("controller/enable", 5, onEnableCtrl);

    ros::Subscriber pidDecreaseSub = 
            node.subscribe("pid/decrease", 5, onPidD);

    ros::Subscriber pidIncreaseSub = 
            node.subscribe("pid/increase", 5, onPidI);

    ros::Subscriber truth = 
            node.subscribe("Robot_1/pose", 1, onTruth);

    control.cmdPublisher =
            node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate r(50); // 50 hz
    while (1)
    {
        ros::spinOnce();
        onControl();
        optitrack = false;
        r.sleep();
    }

    return 0;
}
