//copy from mateo's first node
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

// Vehicle parameters
const double WHEELBASE = 1.765; // Front-to-rear wheel distance (meters)
const double STEERING_FACTOR = 32.0; // Steering factor

// Global variables for odometry computation
double x = 0.0, y = 0.0, theta = 0.0; // Pose (x, y, theta)
ros::Time prev_time; // Previous timestamp

class FrotntRearNode {
private:
    // Subscribers for the front and rear GPS topics
    ros::Subscriber front_gps_sub_;
    ros::Subscriber rear_gps_sub_;

    // Publisher for the odometry topic
    ros::Publisher odom_pub_;

    // Transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;

    // Latest received messages from the subscribed topics
    sensor_msgs::NavSatFix front_gps_msg_;
    sensor_msgs::NavSatFix rear_gps_msg_;
public:
    FrotntRearNode() {
        // Initialize subscribers and publishers
        ros::NodeHandle nh;
        front_gps_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 1, &FrotntRearNode::frontGpsCallback, this);
        rear_gps_sub_ = nh.subscribe("/swiftnav/rear/gps_pose", 1, &FrotntRearNode::rearGpsCallback, this);
    }
}


void frontGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Extract speed (km/h) and steering wheel angle (degrees)
    double front_latitude = msg->latitude;
    double front_longitude = msg->longitude;

    // Convert speed to m/s and steering angle to wheel angle
    double speed_mps = speed_kmh / 3.6;
    double wheel_angle_rad = (steering_wheel_angle_deg / STEERING_FACTOR) * M_PI / 180.0;

    // Compute angular velocity (bicycle model)
    double angular_velocity = (speed_mps / WHEELBASE) * tan(wheel_angle_rad);

    // Calculate time difference
    ros::Time current_time = msg->header.stamp;
    if (prev_time.isZero()) {
        prev_time = current_time;
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
    odom_msg.child_frame_id = "gps";

    // Set position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0; // 2D assumption

    // Compute quaternion using setRPY (roll=0, pitch=0, yaw=theta)
    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, theta);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat); // Convert to geometry_msgs
    odom_msg.pose.pose.orientation = odom_quat;

    // Set velocity
    odom_msg.twist.twist.linear.x = speed_mps;
    odom_msg.twist.twist.angular.z = angular_velocity;

    if (!odom_pub) {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 10);
    }
    odom_pub.publish(odom_msg);

    // Broadcast TF transform (odom → vehicle)
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf_quat); // Reuse the same quaternion
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gps"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer_gps");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/swiftnav/front/gps_pose", 10, frontGpsCallback);
    ros::Subscriber subq = nh.subscribe("/swiftnav/rear/gps_pose", 10, rearGpsCallback);
    FrontRearNode front_rear;
    ros::spin();
    return 0;
}