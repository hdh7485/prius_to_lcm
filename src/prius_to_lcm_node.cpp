#include <ros/ros.h>
#include <tf/tf.h>
#include <prius_msgs/Control.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

#include <lcm/lcm.h>
#include <lcm/lcm-cpp.hpp>

#include "eurecar_lcmtypes/eurecar/can_t.hpp"
#include "eurecar_lcmtypes/eurecar/pos_t.hpp"
#include "eurecar_lcmtypes/eurecar/gga_t.hpp"


class PriusToLCM {
private:
  ros::Subscriber sub_ground_truth_;
  ros::Subscriber sub_prius_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh;

  double x_;
  double y_;
  double yaw_;
  double velocity_;
  double yaw_rate_;

  eurecar::can_t lcm_can_t_;
  eurecar::pos_t lcm_pos_t_;
  eurecar::gga_t lcm_gga_t_;

  double steering_angle_;
  double max_ackermann_a_;
  double max_ackermann_b_;
  double i30_steer_ratio_;

public:
  lcm::LCM lcm;

  PriusToLCM():nh_("~") {
    // init var
    steering_angle_ = 0;
    x_ = 0;
    y_ = 0;
    yaw_ = 0;
    velocity_ = 0;
    yaw_rate_ = 0;

    sub_ground_truth_ = nh.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 100, &PriusToLCM::navMsgCallback, this);
    sub_prius_ = nh.subscribe<prius_msgs::Control>("/prius", 100, &PriusToLCM::priusMsgCallback, this);
    nh_.param("max_ackermann_a", max_ackermann_a_, 43.365);
    nh_.param("max_ackermann_b", max_ackermann_b_, 32.084);
    nh_.param("i30_steer_ratio", i30_steer_ratio_, 17.2783405136);
  }

  void navMsgCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    yaw_ = tf::getYaw(pose.getRotation());

    velocity_ = std::sqrt(std::pow(msg->twist.twist.linear.x, 2) + std::pow(msg->twist.twist.linear.y, 2));
    yaw_rate_ = msg->twist.twist.angular.z;

    //ROS_INFO_STREAM("x:" << x_ << " y:" << y_ << " yaw:" << yaw_ << " vel:" << velocity_ << " yaw_rate:" << yaw_rate_);

    // CAN data assign
    lcm_can_t_.utime = ros::Time::now().toSec();
    lcm_can_t_.time = ros::Time::now().toSec();
    lcm_can_t_.yaw_rate = -1 * yaw_rate_;
    lcm_can_t_.mdps_torque = 0;
    lcm_can_t_.mdps_str_ang = steering_angle_;
    lcm_can_t_.VS_CAN = velocity_ * 3.6;
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

    std::cout << "vel m/s : " << velocity_ << std::endl;
    std::cout << "vel km/h : " << velocity_*3.6 << std::endl;
    std::cout << "yaw : " << yaw_ << std::endl;

    lcm.publish("CAN_T",&lcm_can_t_);

    // POS,GGA data assign
    lcm_pos_t_.utime = ros::Time::now().toSec();
    lcm_pos_t_.dt = 0.01;
    lcm_pos_t_.x = x_;
    lcm_pos_t_.y = -y_;
    lcm_pos_t_.v = 0;
    lcm_pos_t_.h = -1 * yaw_;
    lcm_pos_t_.yaw_rate = -1 * yaw_rate_;

    lcm_gga_t_.utime = ros::Time::now().toSec();
    lcm_gga_t_.x = x_;
    lcm_gga_t_.y = -y_;
    lcm_gga_t_.h = 0;
    lcm_gga_t_.numSV = 0;
    lcm_gga_t_.postype = 0;
    lcm_gga_t_.lag = 0;
    lcm_gga_t_.hordev = 0;
    lcm_gga_t_.lat = 0;
    lcm_gga_t_.lon = 0;
    lcm_gga_t_.hgt = 0;
    lcm_gga_t_.map_idx = 0;

    lcm.publish("POS_T",&lcm_pos_t_);
    lcm.publish("GGA_1",&lcm_gga_t_);
  }

  void priusMsgCallback(const prius_msgs::Control::ConstPtr& msg) {
    steering_angle_ = msg->steer * i30_steer_ratio_ * (max_ackermann_a_ + max_ackermann_b_)/2;
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
