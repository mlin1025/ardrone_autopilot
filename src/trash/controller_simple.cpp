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
#include <geometry_msgs/Twist.h>


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


// Circle output
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
    ros::Publisher command;
    bool enabled = false;
    bool boxCaptured = false;
    Box box;
    std::vector<Circle> targ;
};

struct ControlCenter control;


// Enable/disable controller

void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n"; 
}


// Extract information from incoming message 
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

void pi_reg(char coord, int i) {
    float k = 0.3;
    switch (coord) {
        case 'x': {
            float curr_coord = control.targ[i].x;
            while (curr_coord >= control.box.right) {
                geometry_msgs::Twist msg;
                msg.linear.y = k*((control.box.right - curr_coord)/640);
                control.command.publish(msg);
                curr_coord += -50;
		std::cout << "vel.x = " << msg.linear.y << '\n';
		std::cout << "coord = " << curr_coord << "\n";
		std::cout << "Right goal = " << control.box.right << "\n\n";
            }
            while (curr_coord <= control.box.left) {
                geometry_msgs::Twist msg;
                msg.linear.y = k*((control.box.left - curr_coord)/640);
                control.command.publish(msg);
                curr_coord += 50;
		std::cout << "vel.x = " << msg.linear.y << '\n';
                std::cout << "coord = " << curr_coord << '\n';
		std::cout << "Left goal = " << control.box.left << "\n\n";
            }
            break;
        }
        case 'y': {
            float curr_coord = control.targ[i].y;
            while (curr_coord <= control.box.top) {
                geometry_msgs::Twist msg;
                msg.linear.x = k*((control.box.top - curr_coord)/480);
                control.command.publish(msg);
                curr_coord += 50;
		std::cout << "vel.y = " << msg.linear.x << '\n';
		std::cout << "coord = " << curr_coord << "\n";
		std::cout << "Top goal = " << control.box.top << "\n\n";
            }
            while (curr_coord >= control.box.bottom) {
                geometry_msgs::Twist msg;
                msg.linear.x = k*((control.box.bottom - curr_coord)/480);
                control.command.publish(msg);
                curr_coord += -50;
		std::cout << "vel.y = " << msg.linear.x << '\n';
		std::cout << "coord = " << curr_coord << "\n";
		std::cout << "Bottom goal = " << control.box.bottom << "\n\n";
            }
            break;
        }
    }
}

// Recieving information and controlling the drone
void onTarget(const std_msgs::Float32MultiArray& msg) {
        if (control.enabled) {

            control.targ.clear();
            parseArray(msg);
            // TO DO controller
if (control.targ.size() >= 3) {
            while(!(control.targ[0].inTheBox && control.targ[1].inTheBox && control.targ[2].inTheBox)) {
                if (!control.targ[0].inTheBox) {
                    if (control.targ[0].x < control.box.left || control.targ[0].x > control.box.right) {
                        pi_reg('x', 0);
                    }
                    if (control.targ[0].y < control.box.bottom || control.targ[0].x > control.box.top) {
                        pi_reg('y', 0);
                    }
		control.targ[0].inTheBox = true;
                }
                if (!control.targ[1].inTheBox) {
                    if (control.targ[1].x < control.box.left || control.targ[1].x > control.box.right) {
                        pi_reg('x', 1);
                    }
                    if (control.targ[1].y < control.box.bottom || control.targ[1].x > control.box.top) {
                        pi_reg('y', 1);
                    }
		control.targ[1].inTheBox = true;
                }
                if (!control.targ[2].inTheBox) {
                    if (control.targ[2].x < control.box.left || control.targ[2].x > control.box.right) {
                        pi_reg('x', 2);
                    }
                    if (control.targ[2].y < control.box.bottom || control.targ[2].x > control.box.top) {
                        pi_reg('y', 2);
                    }
                }
		control.targ[2].inTheBox = true;
            }
}


            // Circle information output
            std::cout << "-----------------------\n";
            for (auto& circle : control.targ) {
                std::cout << circle << '\n';
            }
            std::cout << "-----------------------\n";
    }
}



// Extract box information from incoming message
void onBox(const std_msgs::Float32MultiArray& msg) {
        control.box.left = msg.data[0];
        control.box.right = msg.data[1];
        control.box.top = msg.data[2];
        control.box.bottom = msg.data[3];
        control.boxCaptured = true;
}
        

int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller");
    ros::NodeHandle node;
    

    ros::Subscriber boxSub = 
            node.subscribe("box", 1, onBox);

    ros::Subscriber enableSub = 
            node.subscribe("controller/enable", 5, onEnableCtrl);


    ros::Subscriber targetSub = 
            node.subscribe("target", 5, onTarget);


    control.command = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    ros::spin();

    return 0;
}
