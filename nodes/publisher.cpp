#include "ros/ros.h"
#include "std_msgs/String.h"

int main() {
    ros::init("points");
    ros::NodeHandle n;
    ros::Publisher point_send = n.advertise<std_msgs::String>("point", 1000);
    ros::Rate loop_rate(10);
    std_msgs::Stringmsg;
    msg.data = "Hello\n";
    point_send.publish(msg);
    std::cout << "print";
}


