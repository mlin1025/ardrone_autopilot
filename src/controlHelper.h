#include <string>
#include <iostream>
#include "imgHelper.h"
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "tf/LinearMath/Transform.h"
#include <cmath>

/*  
    controlHelper.h
  ----------------------------------------------------------------------------
  | Contains some utility classes and functions for the controller.cpp.      |
  ----------------------------------------------------------------------------

*/

class PID_Controller {
 public:
  PID_Controller() {
    //twist_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //pos_pub =
    //  n.advertise<geometry_msgs::PoseStamped>("ardrone/true_position", 1);
    //vrpn_sub = n.subscribe("vrpn_client_node/ardrone/pose", 1,
    //                       &PID_Controller::vrpn_callback, this);
    //curr_pose_set_sub = n.subscribe("ardrone/current_position", 1,
    //                                &PID_Controller::curr_pose_set_callback, this);

    //ROS_INFO_STREAM("Made subscriber");

    kp[0] = 0.2;
    kp[1] = 0.2;
    kp[2] = 1;
    kp[3] = 1;

    kd[0] = -0.1;
    kd[1] = -0.1;
    kd[2] = 1;
    kd[3] = 0;

    for (int i = 0; i < 3; i++) {
      first_d[i] = true;
      a1[i] = 0.0;
      a0[i] = 0.0;
      t1[i] = 0.0;
      t0[i] = 0.0;
      true_pos[i] = 0.0;
    }

    current_pos[0] = 0;//1;
    current_pos[1] = 0;//-1;
    current_pos[2] = 1;
    current_rot = tf::Quaternion(1, 0, 0, 0);
    true_rot = tf::Quaternion(1, 0, 0, 0);
  }

  void reset(){
    for (int i = 0; i < 3; i++) {
      first_d[i] = true;
      a1[i] = 0.0;
      a0[i] = 0.0;
      t1[i] = 0.0;
      t0[i] = 0.0;
      true_pos[i] = 0.0;
    }
  }
  /**
   * Converts from global to drone coordinates
   */
  void global_to_drone_coordinates(float mat[4], float angle,
                                   float new_mat[4]) {
    new_mat[0] = cos(angle) * mat[0] - sin(angle) * mat[1];
    new_mat[1] = sin(angle) * mat[0] + cos(angle) * mat[1];
    new_mat[2] = mat[2];
    new_mat[3] = mat[3];
  }

  /**
   * Calculates p
   */
  float calc_p_control(float kp, float current_val, float true_val) {
    return kp * (current_val - true_val);
  }

  /**
   * Calculates d
   */
  float calc_d_control(float kd, float current, double time, int axis) {
    float coef1 = 0.9;
    float coef0 = 0.925;

    /* ROS_INFO_STREAM(first_d[axis]); */
    if (first_d[axis]) {
      a1[axis] = current;
      a0[axis] = current;
      t1[axis] = time;
      t0[axis] = time;

      first_d[axis] = false;
    } else {
      a1[axis] = coef1 * a1[axis] + (1.0 - coef1) * current;
      a0[axis] = coef0 * a0[axis] + (1.0 - coef0) * current;
      t1[axis] = coef1 * t1[axis] + (1.0 - coef1) * time;
      t0[axis] = coef0 * t0[axis] + (1.0 - coef0) * time;
    }

    float numerator = a1[axis] - a0[axis];
    float denominator = t1[axis] - t0[axis];

    float val = kd * numerator / denominator;
    if (std::isnan(val)) {
      return 0.0;
    } else {
      return val;
    }
  }

  /**
   * Calculates the offset angle
   */
  float calc_offset_angle(tf::Quaternion c) {
    tf::Quaternion x_axis = tf::Quaternion(1, 0, 0, 0);
    tf::Quaternion a = x_axis * c.inverse();
    tf::Quaternion rotation = (a * x_axis) * a.inverse();
    float angle = atan2(rotation[1], rotation[0]);

    //float angle = - atan2(2*(c[0]*c[3]+c[1]*c[2]), 1-2*(c[2]*c[2]+c[3]*c[3])) -M_PI;
    if (angle < 0.0) {
      angle += 2 * M_PI;
    }
    return angle;
  }

  /**
   * Calculates p for rotation
   */
  float calc_p_control_angle(float kp, float current_rot,
                             tf::Quaternion true_rot) {
    float angle = calc_offset_angle(true_rot);
    float alpha = current_rot - angle;
    if (alpha > M_PI) {
      alpha -= 2 * M_PI;
    }else if( alpha < -M_PI){
      alpha += 2 * M_PI;
    }
    ROS_INFO_STREAM(alpha / M_PI);
    /* float after_p = calc_p_control(kp, 0, alpha); */
    /* return after_p; */
    return alpha * kp;
  }

  /**
   * Caculate that handles driving the drone to the current_pos
   * position
   */
  geometry_msgs::Twist caculate(const geometry_msgs::PoseStamped truth) {
    /* Getting true_pos and rot*/
    true_pos[0] = truth.pose.position.x;
    true_pos[1] = truth.pose.position.y;
    true_pos[2] = truth.pose.position.z;

    true_rot[0] = truth.pose.orientation.x;
    true_rot[1] = truth.pose.orientation.y;
    true_rot[2] = truth.pose.orientation.z;
    true_rot[3] = truth.pose.orientation.w;

    /* Generating twist */
    geometry_msgs::Twist twist_msg;

    /* Arbitrary values to disable hover mode */
    //twist_msg.angular.x = 50;
    //twist_msg.angular.y = 200;

    double roll, pitch, yaw;
    tf::Matrix3x3(current_rot).getRPY(roll, pitch, yaw);

    float wanted_offset_angle = calc_offset_angle(current_rot);
    float p[4] = {calc_p_control(kp[0], current_pos[0], true_pos[0]),
                  calc_p_control(kp[1], current_pos[1], true_pos[1]),
                  calc_p_control(kp[2], current_pos[2], true_pos[2]),
                  calc_p_control_angle(kp[3], wanted_offset_angle, true_rot)};

    double time = truth.header.stamp.toSec();

    float d[4] = {calc_d_control(kd[0], true_pos[0], time, 0),
                  calc_d_control(kd[1], true_pos[1], time, 1), 0, 0};

    float offset_angle = calc_offset_angle(true_rot);

    /* ROS_INFO_STREAM(p[0] << ", " << p[1] << ", " << p[2] << ", " << p[3]); */
    /* ROS_INFO_STREAM(true_pos[0] << ", " << true_pos[1] << ", " << true_pos[2]); */
    /* ROS_INFO_STREAM(current_pos[0] << ", " << current_pos[1] << ", " << current_pos[2]); */

    float rot_p[4], rot_d[4];
    global_to_drone_coordinates(p, offset_angle, rot_p);
    global_to_drone_coordinates(d, offset_angle, rot_d);
    global_to_drone_coordinates(p, offset_angle, rot_p);
    global_to_drone_coordinates(d, offset_angle, rot_d);

    twist_msg.linear.x = rot_p[0] + rot_d[0];
    twist_msg.linear.y = rot_p[1] + rot_d[1];
    twist_msg.linear.z = rot_p[2] + rot_d[2];
    twist_msg.angular.z = rot_p[3] + rot_d[3];

    std::cout<< "y=" << twist_msg.linear.y << " x=" << twist_msg.linear.x << " z=" << twist_msg.linear.z << std::endl;
    /* ROS_INFO_STREAM(twist_msg.angular.z << ", " << offset_angle / M_PI << ", " << wanted_offset_angle / M_PI); */
    return twist_msg;

  }

  void target_pose_set(const geometry_msgs::Pose input_current_pos) {
    current_pos[0] = fmin(1.6, fmax(0.1, input_current_pos.position.x));
    current_pos[1] = fmin(-0.1, fmax(-1.6, input_current_pos.position.y));
    current_pos[2] = fmin(1.6, fmax(0.1, input_current_pos.position.z));
    if (!std::isnan(input_current_pos.orientation.x) && !std::isnan(input_current_pos.orientation.y) && !std::isnan(input_current_pos.orientation.z) && !std::isnan(input_current_pos.orientation.w)) {
      current_rot[0] = input_current_pos.orientation.x;
      current_rot[1] = input_current_pos.orientation.y;
      current_rot[2] = input_current_pos.orientation.z;
      current_rot[3] = input_current_pos.orientation.w;
    }
  }

 private:
  float kp[4];
  float kd[4];

  bool first_d[3];
  float a1[3];
  float a0[3];
  double t1[3];
  double t0[3];

  float current_pos[3];
  tf::Quaternion current_rot;
  float true_pos[3];
  tf::Quaternion true_rot;
};



// PID controller realization.

class PID {
 public:
    float kP, kD, kI;
    float integralX, integralY;
    float prevErrorX, prevErrorY;
    float dt;

    PID(float Dt, float Kp, float Ki, float Kd): 
        dt(Dt),
        kP(Kp), 
        kD(Kd), 
        kI(Ki), 
        prevErrorX(0),
	prevErrorY(0), 
        integralX(0),
        integralY(0) {}

    // PID calculation. Takes the error and the flag.
    //	If x is true, PID calcs ouput for X and for Y otherwise.
    
    float calculate(float error, bool x) {
	float prevError, integral;
	x ? prevError = prevErrorX : prevError = prevErrorY; 
	x ? integral = integralX : integral = integralY; 
        float outP = kP * error;

        integral += error * dt;
        float outI = kI * integral;
        
        float derivative;

        if (prevError != 0) {
            derivative = (error - prevError) / dt;
        } else {
            derivative = 0;
        }

        float outD = kD * derivative;

	x ? integralX = integral : integralY = integral; 
        float output = outP + outI + outD;

	std::cout << "--------------PID------------------\n";
	std::cout << "Error: " << error << '\n';
	std::cout << "prevError: " << prevError << '\n';

	std::cout << "dt: : " << dt << '\n';
	std::cout << "IntegralX: " << integralX << '\n';
	std::cout << "IntegralY: " << integralY << '\n';

	std::cout << "kP: " << kP << "| P: " << outP << '\n';
	std::cout << "kI: " << kI << "| I: " << outI << '\n';
	std::cout << "kD: " << kD << "| D: " << outD << '\n';

	if (x)
	    prevErrorX = error;
	else
	    prevErrorY = error;

	return output;
    }   
     
};


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
        inTheBox(b)
         {}
};

// Main class of the controller. Contains flags, target information, PID controller and other info.

class ControlCenter
{
    public:
        bool enabled;
        float imgRows, imgCols;
        float triCenterX, triCenterY;
        Box box;
        std::vector<Circle> targ;
        ros::Publisher cmdPublisher;
        PID_Controller pid;
        ros::Time lastLoop;
        ControlCenter(){} // Started PID coefficients. Can be changed.
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


