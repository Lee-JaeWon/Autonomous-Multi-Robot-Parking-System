#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>


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
ros::Publisher pub_dp;

//Subscribers
ros::Subscriber sub;
ros::Subscriber sub_trajectory;

//variables
int hz = 33;
bool tf_listened = false;
bool trajectiory_subscribed = false;
bool start_tracking = false;

double distError;
double robot_X;
double robot_Y;
double robot_Yaw;

//Stanley Parameters
double k=2.5;
double k2=1.5;
double v=0.3;


void tf_Listener();

