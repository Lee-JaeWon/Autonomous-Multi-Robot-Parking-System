#include "ros/ros.h" // ROS 기본헤더파일
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

nav_msgs::Odometry odom_data;
nav_msgs::OccupancyGrid map_data;

int height;
int width;
double resolution_,origin_x_, origin_y_;

bool worldTomap(double wx, double wy, unsigned int& mx, unsigned int& my){

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < height && my < width) return true;
  return false;
}

void mapToworld(int mx, int my,double& wx,double& wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;

}


void odomcallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    odom_data.pose.pose.position.x = odom_msg->pose.pose.position.x;
    odom_data.pose.pose.position.y = odom_msg->pose.pose.position.y;
}

void mapcallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg){
    map_data = *map_msg;
    height = map_msg->info.height;
    width = map_msg->info.width;
    resolution_ = map_msg->info.resolution;
    origin_x_ = map_msg->info.origin.position.x;
    origin_y_ = map_msg->info.origin.position.y;

    ROS_INFO("Map_height : %d",height);
    ROS_INFO("Map_width : %d",width);
    ROS_INFO("Map_resolution : %f",resolution_);
    ROS_INFO("Map_origin_x : %f",origin_x_);
    ROS_INFO("Map_origin_y : %f",origin_y_);

    int8_t map[height][width];
    int cnt = 0;
    for (int row = 0 ; row < height ; row ++){
        for(int col = 0 ; col < width ; col++){
            map[row][col] = map_msg->data[cnt];
            cnt++; }
    }
}

int main(int argc, char **argv)// 노드메인함수
{
ros::init(argc, argv, "path_node");
ros::NodeHandle nh; 
ros::Subscriber odom_sub = nh.subscribe("odom",100,odomcallback);
ros::Subscriber map_sub =nh.subscribe("map",100,mapcallback);
ros::spin();

return 0;
}