#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <cmath>


ros::Subscriber laser_sub;
ros::Subscriber cmd_sub;
ros::Publisher cmd_pub;
int hz =33;
double obstacle_dist;
double obstacle_angle;

bool calObstacle(sensor_msgs::LaserScan);
bool obstacle=false;
