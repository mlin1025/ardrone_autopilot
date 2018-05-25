
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
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub;
void onTruth(const nav_msgs::Odometry& msg) {
    static tf::TransformBroadcaster tf_pub;
    geometry_msgs::PoseStamped send;
    send.pose = msg.pose.pose;
    send.header = msg.header;
    pub.publish(send);


    tf::Transform transform;
    transform.setOrigin( tf::Vector3(send.pose.position.x,
                                     send.pose.position.y,
                                     send.pose.position.z));
    tf::Quaternion q;

    quaternionMsgToTF(send.pose.orientation, q);

    transform.setRotation(q);
    ros::Time timestamp(ros::Time::now());
    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, "world", "Robot_1_base_link"));
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "relaySim");
    ros::NodeHandle node;

    ros::Subscriber truth = 
            node.subscribe("ground_truth/state", 1, onTruth);

    pub = node.advertise<geometry_msgs::PoseStamped>("Robot_1/pose", 1000);

    ros::spin();

    return 0;
}
