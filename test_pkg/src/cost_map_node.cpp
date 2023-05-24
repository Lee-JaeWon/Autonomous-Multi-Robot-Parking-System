#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <dwa_local_planner/dwa_planner_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_costmap");
  ros::NodeHandle nh;
  // tf 변환 관련 객체 초기화
  tf::TransformListener tf(ros::Duration(10));
  // costmap 초기화
  costmap_2d::Costmap2DROS costmap("my_costmap", tf);
  dwa_local_planner::DWAPlannerROS dp;
  dp.initialize("my_dwa_planner", &tf, &costmap);
  // LIDAR 데이터를 받을 토픽 설정
  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, boost::bind(&costmap_2d::Costmap2DROS::laserScanCallback, &costmap, _1));

  ros::spin();
  return 0;
}
