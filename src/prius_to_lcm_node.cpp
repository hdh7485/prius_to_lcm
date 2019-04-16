#include <ros/ros.h>
#include <tf/tf.h>
//#include <prius_msgs/
#include <nav_msgs/Odometry.h>
#include <cmath>

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
public:
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
  }
  
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "prius_to_lcm_node");
  PriusToLCM p2l;

  ros::spin();

  return 0;
}
