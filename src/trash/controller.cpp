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


class Circle {
 public:
    float x, y, width, height;
    bool inTheBox;
    
    Circle() {}

    Circle(float _x, float _y, float _width, float _height, bool b):
        x(_x), 
        y(_y), 
        width(_width), 
        height(_height), 
        inTheBox(b) {}

    void setCircle(float _x, float _y, float _width, float _height, bool b)
    {
        x = (_x);
        y = (_y);
        width = (_width);
        height = (_height); 
        inTheBox = (b);
    }
    
};
    std::ostream &operator<<(std::ostream &os, Circle& c) { 
        std::string output;
        output = "X: " + std::to_string(c.x) + '\n';
        output += "Y: " + std::to_string(c.y) + '\n';
        output += "Width: " + std::to_string(c.width) + '\n';
        output += "Height: " + std::to_string(c.height) + '\n';
        output += "InTheBox: " + std::to_string(c.inTheBox) + '\n';
        return os << output;
    }

struct ControlCenter
{
    bool enabled = false;
    bool boxCaptured = false;
    Box box;
    std::vector<Circle> targ;
};

struct ControlCenter control;

void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n"; 
}

/*
void insertCircle(std::vector<float>& data) {
    control.targ.push_back(Circle(data[0], data[1], data[2], data[3], data[4]));
}

void parseTargetString(const std_msgs::String& msg) {
    std::string str(msg.data);
    std::string currStr;
    std::vector<float> circleData;
    size_t counter = 0;
    size_t i = 0;
    while (i != str.size()) {
        if (str[i] != ' ') {
                if (str[i] != ',') {
                    currStr += str[i];
//
                    std::cout << "CurDATA " << currStr << '\n';
                } else {
                    float currData = std::stof(currStr);
                    switch (counter) {
                        case 0 : control.box.left = currData;
                                 break;
                        case 1 : control.box.right = currData;
                                 break;
                        case 2 : control.box.top = currData;
                                 break;
                        case 3 : control.box.bottom = currData;
                                 break;
                        default : circleData.push_back(currData);
                    }
                    ++counter;
                }
        } else if (counter > 5) {
            insertCircle(circleData);
        }
        ++i;
    }
}
*/

void parseArray(const std_msgs::Float32MultiArray& msg) {
    if (msg.data.size() != 0) {
        size_t i = 0;
        Circle circle;
        while (i < msg.data.size()) {
            circle.x = msg.data[i++];
            circle.y = msg.data[i++];
            circle.width = msg.data[i++];
            circle.height = msg.data[i++];
            circle.inTheBox = !msg.data[i++];
            control.targ.push_back(Circle(circle));
        }   
    }   
}
    

void onTarget(const std_msgs::Float32MultiArray& msg) {
        if (control.enabled) {
            //parseTargetString(str);
            control.targ.clear();
            // TO DO controller
            parseArray(msg);
            std::cout << "-----------------------\n";
            for (auto& circle : control.targ) {
                std::cout << circle << '\n';
            }
            std::cout << "-----------------------\n";
    }
}

void onBox(const std_msgs::Float32MultiArray& msg) {
    if (!control.boxCaptured) {
        control.box.left = msg.data[0];
        control.box.right = msg.data[1];
        control.box.top = msg.data[2];
        control.box.bottom = msg.data[3];
        control.boxCaptured = true;
    }
}
        

int main(int argc, char **argv)
{
    /* 
     * Vasya was here
     */

    ros::init(argc, argv, "controller");
    ros::NodeHandle node;
    
    ros::Subscriber boxSub = 
            node.subscribe("box", 1, onBox);

    ros::Subscriber enableSub = 
            node.subscribe("controller/enable", 5, onEnableCtrl);


    ros::Subscriber targetSub = 
            node.subscribe("target", 5, onTarget);

    ros::spin();
//    int count = 0;
//    ++count;


    return 0;
}
