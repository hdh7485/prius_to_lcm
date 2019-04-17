#include <ros/ros.h>
#include <tf/tf.h>
#include <prius_msgs/Control.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

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
  double steering_angle_;
  double max_ackermann_a_;
  double max_ackermann_b_;
  double i30_steer_ratio_;
public:
  PriusToLCM():nh_("~") {
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

    ROS_INFO_STREAM("x:" << x_ << " y:" << y_ << " yaw:" << yaw_ << " vel:" << velocity_ << " yaw_rate:" << yaw_rate_ << "steering_angle:" << steering_angle_);
  }

  void priusMsgCallback(const prius_msgs::Control::ConstPtr& msg) {
    steering_angle_ = msg->steer * i30_steer_ratio_ * (max_ackermann_a_ + max_ackermann_b_)/2;

    //ROS_INFO_STREAM("steering_angle:" << steering_angle_);
  }
  
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "prius_to_lcm_node");
  PriusToLCM p2l;

  ros::spin();

  return 0;
}
