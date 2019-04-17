#include <ros/ros.h>
#include <tf/tf.h>
//#include <prius_msgs/
#include <nav_msgs/Odometry.h>
#include <cmath>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "eurecar_lcmtypes/eurecar/can_t.hpp"
#include "eurecar_lcmtypes/eurecar/pos_t.hpp"


class PriusToLCM {
private:
  ros::Subscriber sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh;

  double x_;
  double y_;
  double yaw_;
  double velocity_;
  double yaw_rate_;

  eurecar::can_t lcm_can_t_;
  eurecar::pos_t lcm_pos_t_;

public:
  lcm::LCM lcm;

  PriusToLCM():nh_("~") {
    sub_ = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 100, &PriusToLCM::navMsgCallback, this);
  }

  void navMsgCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    yaw_ = tf::getYaw(pose.getRotation());

    velocity_ = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
    yaw_rate_ = msg->twist.twist.angular.z;

    ROS_INFO_STREAM("x:" << x_ << " y:" << y_ << " yaw:" << yaw_ << " vel:" << velocity_ << " yaw_rate:" << yaw_rate_);

    // CAN data assign
    lcm_can_t_.utime = ros::Time::now().toSec();
    lcm_can_t_.time = ros::Time::now().toSec();
    lcm_can_t_.yaw_rate = yaw_rate_;
    lcm_can_t_.mdps_torque = 0;
    lcm_can_t_.mdps_str_ang = -5555; // should be parsed 
    lcm_can_t_.VS_CAN = velocity_;
    lcm_can_t_.lat_accel = 0;
    lcm_can_t_.mcp = 0;
    lcm_can_t_.accel_pedal_value = 0;
    lcm_can_t_.tps = 0;
    lcm_can_t_.odometer = 0;
    lcm_can_t_.battery_voltage = 0;
    lcm_can_t_.WHL_SPD_RR = 0;
    lcm_can_t_.WHL_SPD_RL = 0;
    lcm_can_t_.WHL_SPD_FR = 0;
    lcm_can_t_.WHL_SPD_FL = 0;

    lcm.publish("CAN_T",&lcm_can_t_);

    // POS data assign
    lcm_pos_t_.utime = ros::Time::now().toSec();
    lcm_pos_t_.dt = 0.01;
    lcm_pos_t_.x = x_;
    lcm_pos_t_.y = y_;
    lcm_pos_t_.v = 0;
    lcm_pos_t_.h = 0;
    lcm_pos_t_.yaw_rate = yaw_rate_;

    lcm.publish("POS_T",&lcm_pos_t_);
  }
  
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "prius_to_lcm_node");
  PriusToLCM p2l;

  if(!p2l.lcm.good())
	return 1;

  ros::spin();

  return 0;
}
