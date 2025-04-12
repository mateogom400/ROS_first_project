#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <armadillo>  
#include <tf/transform_broadcaster.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // Extract pose using Armadillo
  arma::vec position(3);  // Create a 3D vector
  position(0) = msg->pose.pose.position.x;
  position(1) = msg->pose.pose.position.y;
  position(2) = msg->pose.pose.position.z;
  
  // Broadcast TF
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(position(0), position(1), position(2)));
  transform.setRotation(tf::Quaternion(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  ));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_tf_broadcaster");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("odom", 10, odomCallback);
  ros::spin();
  return 0;
}
