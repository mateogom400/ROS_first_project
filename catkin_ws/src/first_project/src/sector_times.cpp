#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include <cmath>
#include "first_project/sector_time.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>



const double c1_lat=45.63005632834078, c1_lon=9.288958312619226;
const double c2_lat=45.62408441506059, c2_lon=9.288074655564982;
const double start_lat=45.61893210546475, start_lon=9.281178770655666;
const double eps = 0.0005;

int current_sector=3;
ros::Time sector_start_time;
double speed_sum=0;
int count=0;

ros::Publisher sector_pub;

void callback(const geometry_msgs::PointStamped::ConstPtr& speed_msg, const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {


      double lat = (gps_msg->latitude);
      double lon = (gps_msg->longitude);
      double alt = (gps_msg->altitude);

      double speed = (speed_msg->point.y);
      double current_sector_time;
      double current_sector_mean_speed;

      if((lat>start_lat-eps && lat<start_lat+eps) && (lon>start_lon-eps && lon<start_lon+eps) && current_sector==3){
        current_sector = 1;
        speed_sum=0;
        count=1;
        sector_start_time = ros::Time::now();
      //  ROS_INFO("Sono in sector1");
      }
      if ((lat>c1_lat-eps && lat<c1_lat+eps) && (lon>c1_lon-eps && lon<c1_lon+eps) && current_sector==1){
        current_sector = 2;
        speed_sum=0;
        count=1;
        sector_start_time = ros::Time::now();
      //  ROS_INFO("Sono in sector2");
      }
      if((lat>c2_lat-eps && lat<c2_lat+eps) && (lon>c2_lon-eps && lon<c2_lon+eps) && current_sector==2){
        current_sector = 3;
        speed_sum=0;
        count=1;
        sector_start_time = ros::Time::now();
      //  ROS_INFO("Sono in sector3");
      }

      speed_sum+=speed;
      current_sector_mean_speed = speed_sum/count;
      count++;

      current_sector_time = (ros::Time::now()-sector_start_time).toSec();


      first_project::sector_time sector_msg;

      sector_msg.current_sector = current_sector;
      sector_msg.current_sector_time = current_sector_time;
      sector_msg.current_sector_mean_speed = current_sector_mean_speed;

      sector_pub.publish(sector_msg);
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "third_node");
    ros::NodeHandle nh;

    sector_pub = nh.advertise<first_project::sector_time>("sector_times", 10);

    message_filters::Subscriber<geometry_msgs::PointStamped> speed_sub(nh,"/speedsteer", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh,"/swiftnav/front/gps_pose", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PointStamped, sensor_msgs::NavSatFix> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), speed_sub, gps_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
    return 0;
  }
