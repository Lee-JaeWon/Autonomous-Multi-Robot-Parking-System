
#include "../include/obstacle.h"

sensor_msgs::LaserScan laser_data;
geometry_msgs::Twist velocity_data;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  laser_data = *msg;
}

void velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  velocity_data = *msg;
}

bool calObstacle(sensor_msgs::LaserScan laser_msg)
{
  double lidar_resol;
  int lidar_size = laser_msg.ranges.size();
  lidar_resol = laser_msg.angle_increment * 180 / M_PI; // 각해상도 deg
  double current_angle = 0.0;
  for (int i = 0; i < lidar_size; i++)
  {
    current_angle += lidar_resol;
    if (current_angle < obstacle_angle / 2.0 || 360.0 - obstacle_angle / 2.0 < current_angle)
      if (laser_msg.ranges[i] < obstacle_dist)
      {
        return true;
      }
  }

  // ROS_INFO("resol = %f", lidar_resol);
  // ROS_INFO(" ANGLE = %f", current_angle);

  return false;
}

int main(int argc, char **argv)
{
  // ROS 초기화 및 노드 핸들 생성

  ros::init(argc, argv, "obstacle");
  ros::NodeHandle nh;
  ros::Rate rate(hz);
  ros::NodeHandle n("~");
  // get Parameters

  n.getParam("OBSTACLE_DIST", obstacle_dist);
  n.getParam("OBSTACLE_ANGLE", obstacle_angle);

  laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);
  cmd_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, velCallback);
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_final", 1);

  while (ros::ok())
  {

    obstacle = calObstacle(laser_data);
    if (obstacle)
    {
      ROS_INFO("obstacle is true");
      ROS_INFO("linear.x = %f", velocity_data.linear.x);
      ROS_INFO("angular.z = %f", velocity_data.angular.z);
      velocity_data.linear.x = 0.0;
      velocity_data.angular.z = 0.0;
      cmd_pub.publish(velocity_data);
    }
    else
    {
      ROS_INFO("obstacle is false");
      ROS_INFO("linear.x = %f", velocity_data.linear.x);
      ROS_INFO("angular.z = %f", velocity_data.angular.z);
      cmd_pub.publish(velocity_data);
    }
    cmd_pub.publish(velocity_data);

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}