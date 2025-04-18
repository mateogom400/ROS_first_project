//copy from mateo's first node
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>

// Vehicle parameters
const double WHEELBASE = 1.765; // Front-to-rear wheel distance (meters)
const double STEERING_FACTOR = 32.0; // Steering factor

// Global variables for odometry computation
double x = 0.0, y = 0.0, z =0.0, theta = 0.0; // Pose (x, y, theta)
double delta_x = 0.0, delta_y = 0.0;
double prev_x = 0.0, prev_y = 0.0;
double a = 6378137, b =6356752, e_squared = 1-pow(b,2)/pow(a,2); //semi major and semi minor axis of the earth and eccentricity
ros::Time prev_time; // Previous timestamp

// Origin coordinates and ECEF reference point
double origin_latitude = 0.0, origin_longitude = 0.0, origin_altitude = 0.0;
double x_origin = 0.0, y_origin = 0.0, z_origin = 0.0;

class FrontRearNode {
private:
    // Subscribers for the front and rear GPS topics
    ros::Subscriber front_gps_sub_;
    //ros::Subscriber rear_gps_sub_;

    // Publisher for the odometry topic
    ros::Publisher odom_pub_;

    // Transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;

    // Latest received messages from the subscribed topics
    sensor_msgs::NavSatFix front_gps_msg_;
    //sensor_msgs::NavSatFix rear_gps_msg_;
    tf::Quaternion tf_quat;
    tf::Transform transform;
    ros::NodeHandle nh;
public:
    FrontRearNode() {
        // Initialize subscribers and publishers
        front_gps_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 1, &FrontRearNode::frontGpsCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);
        //rear_gps_sub_ = nh.subscribe("/swiftnav/rear/gps_pose", 1, &FrontRearNode::rearGpsCallback, this);
    }



void frontGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) { 
    //extract data from the message
    double front_latitude = msg->latitude * M_PI/180; // Convert latitude to radians
    double front_longitude = msg->longitude * M_PI/180; // Convert longitude to radians
    double front_altitude = msg->altitude; // Altitude is a linear measurement, no conversion needed
    ros::Time current_time = msg->header.stamp;
    if (prev_time.isZero()) {
        // Initialize the origin
        origin_latitude = front_latitude;
        origin_longitude = front_longitude;
        origin_altitude = front_altitude;
        double N = a / (sqrt(1 - e_squared * pow(sin(origin_latitude), 2)));
        x_origin = (N + origin_altitude) * cos(origin_latitude) * cos(origin_longitude);
        y_origin = (N + origin_altitude) * sin(origin_longitude) * cos(origin_latitude);
        z_origin = (N * (1-e_squared) + origin_altitude) * sin(origin_latitude)*0;
        // return;
    }

    // Convert latitude-longitude to ECEF coordinates
    double N = a/(sqrt(1-e_squared*pow((sin(front_latitude)),2)));
    double x_ecef = (N + front_altitude) * cos(front_latitude) * cos(front_longitude);
    double y_ecef = (N + front_altitude) * sin(front_longitude) * cos(front_latitude);
    double z_ecef = (N * (1-e_squared) + front_altitude) * sin(front_latitude)*0;

    //convert latitude-longitude to ENU coordinates
    x = -sin(origin_longitude) * (x_ecef - x_origin) + cos(origin_longitude) * (y_ecef - y_origin);
    y = -sin(origin_latitude) * cos(origin_longitude) * (x_ecef - x_origin) - sin(origin_longitude) * sin(origin_latitude) * (y_ecef - y_origin) + cos(origin_latitude) * (z_ecef - z_origin);
    z = cos(origin_latitude) * cos(origin_longitude) * (x_ecef - x_origin) + cos(origin_latitude) * sin(origin_longitude) * (y_ecef - y_origin) + sin(origin_latitude) * (z_ecef - z_origin);
    x = x/100;
    y = y/100;
    z = 0;

    // Compute angular velocity (bicycle model)
    //double angular_velocity = (speed_mps / WHEELBASE) * tan(wheel_angle_rad);

    // Calculate time difference
    if (prev_time.isZero()) {
        prev_time = current_time;
        return;
    }
    double dt = (current_time - prev_time).toSec();
    prev_time = current_time;

    // Update pose using Euler integration //(we don't need that?)
    //x += speed_mps * cos(theta) * dt;
    //y += speed_mps * sin(theta) * dt;
    //theta += angular_velocity * dt;

    //static tf::TransformBroadcaster tf_broadcaster;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "vehicle";

    // Set position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z; // 2D assumption but we have altitude infos so okayyyy
    if (prev_time.isZero()) {
        delta_x = 0;
        delta_y = 0;
        return;
    }else{
        delta_x = x - prev_x;
        delta_y = y - prev_y;
    }
    theta = atan2(delta_y, delta_x); // Compute theta from delta_x and delta_y
    prev_x = x;
    prev_y = y;
    // Compute quaternion using setRPY (roll=0, pitch=0, yaw=theta)
    
    // //Update the transform's origin with the new pose
    // transform_.setOrigin(tf::Vector3(x, y, z));
    // //Transform to Vehicle frame via tf
    // transform_.setRotation(tf_quat);
    // // Publish the updated transform
    // tf_broadcaster_.sendTransform(tf::StampedTransform(transform_, current_time, "odom", "vehicle"));
        // Broadcast TF transform (odom → vehicle)
    transform.setOrigin(tf::Vector3(x, y, z));
    tf_quat.setRPY(0, 0, theta); 
    transform.setRotation(tf_quat); // Reuse the same quaternion
    //geometry_msgs::Quaternion odom_quat;
    //tf::quaternionTFToMsg(tf_quat, odom_quat); // Convert to geometry_msgs
    odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta); // Set orientation using yaw angle


    
    // Set velocity //(should we set it?)
    //odom_msg.twist.twist.linear.x = speed_mps;
    //odom_msg.twist.twist.angular.z = angular_velocity;

    // if (!odom_pub) {
    //     ros::NodeHandle nh;
    //     odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    // }
    odom_pub_.publish(odom_msg);
    // Publish the transform
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, current_time, "odom", "vehicle"));
    //tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle"));
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odom");
    FrontRearNode front_rear;
    ros::spin();
    return 0;
}