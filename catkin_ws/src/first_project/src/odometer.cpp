#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

// Vehicle parameters
const double WHEELBASE = 1.765; // Front-to-rear wheel distance (meters)
const double STEERING_FACTOR = 32.0; // Steering factor
const double ANGULAR_VELOCITY_EPSILON = 1e-6; // Threshold for w ≈ 0

// Global variables for odometry computation
double x = 0.0, y = 0.0, theta = 0.0;
ros::Time prev_time;

void speedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // Extract data
    double speed_kmh = msg->point.y;
    double steering_wheel_angle_deg = msg->point.x;

    // Convert units
    double speed_mps = speed_kmh / 3.6;
    double wheel_angle_rad = (steering_wheel_angle_deg / STEERING_FACTOR) * M_PI / 180.0;
    double angular_velocity = (speed_mps / WHEELBASE) * tan(wheel_angle_rad);

    // Time handling
    ros::Time current_time = msg->header.stamp;
    if (prev_time.isZero()) {
        prev_time = current_time;
        return;
    }
    double dt = (current_time - prev_time).toSec();
    prev_time = current_time;

    // ********************** INTEGRATION METHOD CHANGE **********************
    double delta_theta = angular_velocity * dt;
    
    if (fabs(angular_velocity) > ANGULAR_VELOCITY_EPSILON) {
        // Exact integration (circular motion)
        double radius = speed_mps / angular_velocity;
        x += radius * (sin(theta + delta_theta) - sin(theta));
        y += radius * (cos(theta) - cos(theta + delta_theta));
    } else {
        // Runge-Kutta approximation (θ_k+1/2 = θ_k + ω_k * T_s / 2)
        double theta_mid = theta + angular_velocity * dt / 2.0;
        x += speed_mps * cos(theta_mid) * dt;
        y += speed_mps * sin(theta_mid) * dt;
    }
    theta += delta_theta; // Orientation update (same for both methods)
    // ***********************************************************************

    // Publish odometry data (unchanged)
    static ros::Publisher odom_pub;
    static tf::TransformBroadcaster tf_broadcaster;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "vehicle";

    // Position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;

    // Orientation using setRPY
    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, theta);
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(tf_quat, odom_quat);
    odom_msg.pose.pose.orientation = odom_quat;

    // Velocity
    odom_msg.twist.twist.linear.x = speed_mps;
    odom_msg.twist.twist.angular.z = angular_velocity;

    if (!odom_pub) {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
    }
    odom_pub.publish(odom_msg);

    // TF Broadcast
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf_quat);
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/speedsteer", 1, speedSteerCallback);
    ros::spin();
    return 0;
}
