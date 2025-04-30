#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <Eigen/Dense>

class GpsOdometer {
private:
    // ROS node handle
    ros::NodeHandle nh_;
    
    // Subscriber and publisher
    ros::Subscriber gps_sub_;
    ros::Publisher odom_pub_;
    
    // Transform broadcaster
    tf::TransformBroadcaster tf_broadcaster_;
    
    // Constants
    const double a = 6378137.0;  // WGS-84 semi-major axis (m)
    const double b = 6356752.0;  // WGS-84 semi-minor axis (m)
    const double e2 = 1.0 - (b*b)/(a*a);  // Square of eccentricity
    
    // Reference coordinates
    double lat_ref_, lon_ref_, alt_ref_;
    Eigen::Vector3d ecef_ref_position_;
    
    // Previous state variables
    bool first_message_ = true;
    ros::Time prev_time_;
    Eigen::Vector3d prev_enu_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d denu_position_ = Eigen::Vector3d::Zero();
    double prev_heading_angle_ = 0.0;
    
    // Smoothing variables
    double alpha_ = 0.8;  // Smoothing factor (0 < alpha < 1)
    double speed_ = 0.0;
    
    // Tunnel detection constants
    const double TUNNEL_X = -1028663.637769047;
    const double TUNNEL_Y = -4477422.006266854;
    
public:
    GpsOdometer() {
        // Get reference position parameters
        if (!getRefPosition()) {
            ROS_WARN("Using default reference position");
        }
        
        // Set up subscriber and publisher
        gps_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 10, &GpsOdometer::gpsCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10);
        
        ROS_INFO("GPS Odometer node initialized");
    }
    
    bool getRefPosition() {
        bool success = true;
        ros::NodeHandle private_nh("~"); // Private node handle to access parameters in this node's namespace
        
        // Print all parameters on the parameter server for debugging
        ROS_INFO("Listing all parameters on the server:");
        std::vector<std::string> params;
        if (ros::param::getParamNames(params)) {
            for (const auto& param : params) {
                ROS_INFO("  Parameter: %s", param.c_str());
            }
        }
        
        // Try different ways to get the parameters
        // First try with private nodehandle (~/param)
        if (private_nh.getParam("lat_r", lat_ref_)) {
            ROS_INFO("Found lat_r in private namespace: %f", lat_ref_);
        } 
        // Try with node namespace (node_name/param)
        else if (nh_.getParam("lat_r", lat_ref_)) {
            ROS_INFO("Found lat_r in node namespace: %f", lat_ref_);
        }
        // Try with global namespace (/param)
        else if (ros::param::get("/lat_r", lat_ref_)) {
            ROS_INFO("Found lat_r in global namespace: %f", lat_ref_);
        }
        // Try with specific namespace (/node_name/param) 
        else if (ros::param::get("/gps_odometer/lat_r", lat_ref_)) {
            ROS_INFO("Found lat_r in /gps_odometer/ namespace: %f", lat_ref_);
        }
        else {
            ROS_WARN("Parameter 'lat_r' not found anywhere, using default value 0.0");
            lat_ref_ = 0.0;
            success = false;
        }
        
        // Same for longitude
        if (private_nh.getParam("lon_r", lon_ref_)) {
            ROS_INFO("Found lon_r in private namespace: %f", lon_ref_);
        } 
        else if (nh_.getParam("lon_r", lon_ref_)) {
            ROS_INFO("Found lon_r in node namespace: %f", lon_ref_);
        }
        else if (ros::param::get("/lon_r", lon_ref_)) {
            ROS_INFO("Found lon_r in global namespace: %f", lon_ref_);
        }
        else if (ros::param::get("/gps_odometer/lon_r", lon_ref_)) {
            ROS_INFO("Found lon_r in /gps_odometer/ namespace: %f", lon_ref_);
        }
        else {
            ROS_WARN("Parameter 'lon_r' not found anywhere, using default value 0.0");
            lon_ref_ = 0.0;
            success = false;
        }
        
        // Same for altitude
        if (private_nh.getParam("alt_r", alt_ref_)) {
            ROS_INFO("Found alt_r in private namespace: %f", alt_ref_);
        } 
        else if (nh_.getParam("alt_r", alt_ref_)) {
            ROS_INFO("Found alt_r in node namespace: %f", alt_ref_);
        }
        else if (ros::param::get("/alt_r", alt_ref_)) {
            ROS_INFO("Found alt_r in global namespace: %f", alt_ref_);
        }
        else if (ros::param::get("/gps_odometer/alt_r", alt_ref_)) {
            ROS_INFO("Found alt_r in /gps_odometer/ namespace: %f", alt_ref_);
        }
        else {
            ROS_WARN("Parameter 'alt_r' not found anywhere, using default value 0.0");
            alt_ref_ = 0.0;
            success = false;
        }
        
        // Convert latitude and longitude to radians
        lat_ref_ = lat_ref_ * M_PI / 180.0;
        lon_ref_ = lon_ref_ * M_PI / 180.0;
        // Note: altitude is already in meters, no conversion needed
        
        // Calculate reference ECEF coordinates
        double N_ref = a / sqrt(1.0 - e2 * pow(sin(lat_ref_), 2));
        ecef_ref_position_(0) = (N_ref + alt_ref_) * cos(lat_ref_) * cos(lon_ref_);
        ecef_ref_position_(1) = (N_ref + alt_ref_) * cos(lat_ref_) * sin(lon_ref_);
        ecef_ref_position_(2) = (N_ref * (1.0 - e2) + alt_ref_) * sin(lat_ref_);
        
        ROS_INFO("Reference position set - Lat: %.6f, Lon: %.6f, Alt: %.2f", 
                 lat_ref_ * 180.0 / M_PI, lon_ref_ * 180.0 / M_PI, alt_ref_);
        
        return success;
    }
    
    double clamp(double val, double min_val, double max_val) {
        return std::max(min_val, std::min(val, max_val));
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Get time from message
        ros::Time curr_time = msg->header.stamp;
        
        // Convert GPS data (LLA) to ECEF
        double lat = msg->latitude * M_PI / 180.0;
        double lon = msg->longitude * M_PI / 180.0;
        double alt = msg->altitude;  // Already in meters
        
        double N = a / sqrt(1.0 - e2 * pow(sin(lat), 2));
        
        Eigen::Vector3d ecef_position;
        ecef_position(0) = (N + alt) * cos(lat) * cos(lon);
        ecef_position(1) = (N + alt) * cos(lat) * sin(lon);
        ecef_position(2) = (N * (1.0 - e2) + alt) * sin(lat);
        
        // Convert ECEF to ENU
        Eigen::Matrix3d R;
        R << -sin(lon_ref_), cos(lon_ref_), 0,
             -sin(lat_ref_) * cos(lon_ref_), -sin(lat_ref_) * sin(lon_ref_), cos(lat_ref_),
             cos(lat_ref_) * cos(lon_ref_), cos(lat_ref_) * sin(lon_ref_), sin(lat_ref_);
        
        Eigen::Vector3d enu_position = R * (ecef_position - ecef_ref_position_);
        
        // Check if in tunnel (bad GPS data) - use estimated position
        if (fabs(enu_position(0) - TUNNEL_X) < 1.0 && fabs(enu_position(1) - TUNNEL_Y) < 1.0) {
            enu_position(0) = prev_enu_position_(0) + denu_position_(0);
            enu_position(1) = prev_enu_position_(1) + denu_position_(1);
            enu_position(2) = prev_enu_position_(2) + denu_position_(2);
            ROS_DEBUG("In tunnel - using estimated position");
        } else {
            // Update displacement only when we have valid GPS data
            denu_position_ = enu_position - prev_enu_position_;
        }
        
        // Calculate speed and heading
        double dt = 0.0;
        double heading_angle = prev_heading_angle_;  // Default to previous heading
        
        if (!first_message_) {
            dt = (curr_time - prev_time_).toSec();
            
            if (dt > 0.0) {
                // Calculate speed
                speed_ = sqrt(pow(denu_position_(0), 2) + pow(denu_position_(1), 2)) / dt;
                
                // Calculate heading angle when moving
                if (speed_ > 0.3) {  // Only update heading when moving significantly
                    heading_angle = atan2(denu_position_(0), denu_position_(1));
                    
                    // Adjust smoothing factor based on speed
                    alpha_ = clamp(1.0 - speed_, 0.5, 0.9);
                    
                    // Apply smoothing filter
                    heading_angle = alpha_ * prev_heading_angle_ + (1.0 - alpha_) * heading_angle;
                    
                    ROS_DEBUG("Speed: %.2f m/s, Heading: %.2f deg", 
                              speed_, heading_angle * 180.0 / M_PI);
                }
            }
        } else {
            first_message_ = false;
        }
        
        // Create and publish odometry message
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = curr_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "gps";
        
        // Set position
        odom_msg.pose.pose.position.x = enu_position(0);
        odom_msg.pose.pose.position.y = enu_position(1);
        odom_msg.pose.pose.position.z = enu_position(2);
        
        // Set orientation
        tf::Quaternion q;
        q.setRPY(0, 0, heading_angle);
        tf::quaternionTFToMsg(q, odom_msg.pose.pose.orientation);
        
        // Set velocity if we have valid time difference
        if (dt > 0.0) {
            odom_msg.twist.twist.linear.x = denu_position_(0) / dt;
            odom_msg.twist.twist.linear.y = denu_position_(1) / dt;
            odom_msg.twist.twist.linear.z = denu_position_(2) / dt;
            odom_msg.twist.twist.angular.z = (heading_angle - prev_heading_angle_) / dt;
        } else {
            // Zero velocity for first message or time issues
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.linear.z = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
        
        // Publish odometry message
        odom_pub_.publish(odom_msg);
        
        // Broadcast transform
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(enu_position(0), enu_position(1), enu_position(2)));
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, curr_time, "odom", "gps"));
        
        // Update previous values for next iteration
        prev_enu_position_ = enu_position;
        prev_time_ = curr_time;
        prev_heading_angle_ = heading_angle;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer gps_odometer;
    ros::spin();
    return 0;
}