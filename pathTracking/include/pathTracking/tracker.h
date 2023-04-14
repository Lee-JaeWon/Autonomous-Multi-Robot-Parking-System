#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Bool.h>

//Messages
nav_msgs::Odometry  state;
nav_msgs::Path path;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::Twist cmd_vel;

//tf2
tf2_ros::Buffer tfBuffer;

//Publishers
ros::Publisher pub;
ros::Publisher pub_vel;

//Publishers-rviz
ros::Publisher pub_dp;
ros::Publisher pub_left_traj;

//Subscribers
ros::Subscriber sub;
ros::Subscriber sub_trajectory;
ros::Subscriber emer_sub;

//variables
bool tf_listened = false;
bool trajectiory_subscribed = false;
bool start_tracking = false;

double distError;
double robot_X;
double robot_Y;
double robot_Yaw;

//Hz Paramaeter
int hz = 33;
std::string tracker_name = "Stanley";

//Stanley Parameters
double k = 4.0;
double k2 = 1.5;
double v = 0.22;

//Kanayama Parameters
double k_x = 0;
double k_y = 0;
double k_theta = 0;
double timeStep = 0.3;

//Stanley + PID Parameters
double ph = 1.8;
double ih = 0.0;
double dh = 1.3;
//
double pc = 1.8;
double ic = 0.0;
double dc = 1.5;

//emergency
bool emer_flag = true;


void pathCallback(const nav_msgs::Path::ConstPtr& path_);
void tf_Listener();

