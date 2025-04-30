#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>

class GpsOdometer {
private:
    ros::Subscriber gps_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // WGS84 parameters
    const double a_ = 6378137.0;       // Semi-major axis [m]
    const double b_ = 6356752.0;       // Semi-minor axis [m]
    const double e2_ = 1 - (b_*b_)/(a_*a_);  // Square of eccentricity

    // Reference point
    double lat_ref_ = 0.0, lon_ref_ = 0.0, alt_ref_ = 0.0;
    double x_ref_ecef_ = 0.0, y_ref_ecef_ = 0.0, z_ref_ecef_ = 0.0;
    bool ref_initialized_ = false;

    // State variables
    Eigen::Vector3d prev_enu_ = Eigen::Vector3d::Zero();
    double prev_heading_ = 0.0;
    ros::Time prev_time_;
    bool first_message_ = true;

    double N_lat(double lat) {
        return a_ / sqrt(1 - e2_ * pow(sin(lat), 2));
    }

    void llaToEcef(double lat, double lon, double alt, 
                  double& x, double& y, double& z) {
        double N = N_lat(lat);
        x = (N + alt) * cos(lat) * cos(lon);
        y = (N + alt) * cos(lat) * sin(lon);
        z = (N * (1 - e2_) + alt) * sin(lat);
    }

public:
    GpsOdometer() {
        ros::NodeHandle nh;
        gps_sub_ = nh.subscribe("/swiftnav/front/gps_pose", 10, &GpsOdometer::gpsCallback, this);
        odom_pub_ = nh.advertise<nav_msgs::Odometry>("/gps_odom", 10);

        // Load reference point from ROS params
        nh.param("/lat_r", lat_ref_, 0.0);
        nh.param("/lon_r", lon_ref_, 0.0);
        nh.param("/alt_r", alt_ref_, 0.0);

        // Convert to radians and compute ECEF reference
        double lat_rad = lat_ref_ * M_PI/180.0;
        double lon_rad = lon_ref_ * M_PI/180.0;
        llaToEcef(lat_rad, lon_rad, alt_ref_, x_ref_ecef_, y_ref_ecef_, z_ref_ecef_);
        ref_initialized_ = true;
        ROS_INFO("Reference point initialized");
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        if (!ref_initialized_) return;

        // Convert current GPS to ECEF
        double lat = msg->latitude * M_PI/180.0;
        double lon = msg->longitude * M_PI/180.0;
        double alt = msg->altitude;
        double x_ecef, y_ecef, z_ecef;
        llaToEcef(lat, lon, alt, x_ecef, y_ecef, z_ecef);

        // ECEF to ENU conversion
        double dx = x_ecef - x_ref_ecef_;
        double dy = y_ecef - y_ref_ecef_;
        double dz = z_ecef - z_ref_ecef_;

        double sin_lon = sin(lon_ref_ * M_PI/180.0);
        double cos_lon = cos(lon_ref_ * M_PI/180.0);
        double sin_lat = sin(lat_ref_ * M_PI/180.0);
        double cos_lat = cos(lat_ref_ * M_PI/180.0);

        Eigen::Vector3d enu;
        enu << -sin_lon * dx + cos_lon * dy,
               -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
               cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;

        // Handle outliers (e.g., tunnels)
        if (enu.head<2>().norm() < 1e-6 || (enu - prev_enu_).norm() > 10.0) {
            enu = prev_enu_;
            ROS_WARN_THROTTLE(1, "GPS outlier detected! Using last valid position.");
        }

        // Compute speed and heading
        ros::Time curr_time = ros::Time::now();
        double dt = (curr_time - prev_time_).toSec();
        double heading = prev_heading_;

        if (dt > 0.001) {  // Valid timestep
            Eigen::Vector2d delta_xy = enu.head<2>() - prev_enu_.head<2>();
            double speed = delta_xy.norm() / dt;

            if (speed > 0.3) {  // Only update heading if moving
                double raw_heading = atan2(delta_xy[1], delta_xy[0]);
                double alpha = std::max(0.5, std::min(0.9, 1.0 - speed));
                heading = alpha * prev_heading_ + (1 - alpha) * raw_heading;
            }
        }

        // Publish odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "gps";
        odom_msg.pose.pose.position.x = enu[0];
        odom_msg.pose.pose.position.y = enu[1];
        odom_msg.pose.pose.position.z = enu[2];
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

        if (dt > 0.001) {
            odom_msg.twist.twist.linear.x = (enu[0] - prev_enu_[0]) / dt;
            odom_msg.twist.twist.linear.y = (enu[1] - prev_enu_[1]) / dt;
        }
        odom_pub_.publish(odom_msg);

        // Broadcast TF
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(enu[0], enu[1], enu[2]));
        tf::Quaternion q;
        q.setRPY(0, 0, heading);
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "gps"));

        // Update state
        prev_enu_ = enu;
        prev_heading_ = heading;
        prev_time_ = curr_time;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odometer");
    GpsOdometer gps_odom;
    ros::spin();
    return 0;
}