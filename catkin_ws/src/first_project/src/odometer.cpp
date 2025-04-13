#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

// Vehicle parameters
const double WHEELBASE = 1.765; // Distance between front and rear wheels (meters)
const double STEERING_FACTOR = 32.0; // Steering factor to convert steering wheel angle to wheel angle

// Global variables for odometry computation
double x = 0.0, y = 0.0, theta = 0.0; // Robot pose (x, y, orientation)
ros::Time prev_time; // Previous timestamp for integration

void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Extract speed (km/h) and steering wheel angle (degrees) from the message
    double speed_kmh = msg->point.y;
    double steering_wheel_angle_deg = msg->point.x;

    // Convert speed to m/s
    double speed_mps = speed_kmh / 3.6;

    // Convert steering wheel angle to wheel angle using steering factor
    double wheel_angle_rad = (steering_wheel_angle_deg / STEERING_FACTOR) * M_PI / 180.0;

    // Compute angular velocity using the bicycle model
    double angular_velocity = (speed_mps / WHEELBASE) * tan(wheel_angle_rad);

    // Get current time and compute time difference
    ros::Time current_time = msg->header.stamp;
    if (prev_time.isZero()) {
        prev_time = current_time; // Initialize previous time on first callback
        return;
    }
    double dt = (current_time - prev_time).toSec();
    prev_time = current_time;

    // Update pose using Euler integration
    x += speed_mps * cos(theta) * dt;
    y += speed_mps * sin(theta) * dt;
    theta += angular_velocity * dt;

    // Publish odometry data
    static ros::Publisher odom_pub;
    static tf::TransformBroadcaster tf_broadcaster;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "vehicle";

    // Set position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0; // Assuming flat terrain

    // Set orientation as quaternion
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;
    odom_quat.z = sin(theta / 2.0);
    odom_quat.w = cos(theta / 2.0);
    odom_msg.pose.pose.orientation = odom_quat;

    // Set velocity
    odom_msg.twist.twist.linear.x = speed_mps;
    odom_msg.twist.twist.linear.y = 0.0; // No lateral movement in nonholonomic model
    odom_msg.twist.twist.angular.z = angular_velocity;

    if (!odom_pub) {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    }
    
    odom_pub.publish(odom_msg);

    // Broadcast TF transform from "odom" to "vehicle"
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    
	tf::Quaternion quat(tf::Vector3(0, theta));
	tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros:
