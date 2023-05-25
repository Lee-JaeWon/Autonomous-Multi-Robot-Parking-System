#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

int calc_current_index(double x , double y ,const nav_msgs::Path& gPath );
int calc_target_index(double mx, double my , const nav_msgs::Path& gPath, int curIdx);
geometry_msgs::Quaternion calc_target_quat(double cx, double cy , double nx, double ny);
double calc_dist(double x , double y , double nx  , double ny);

nav_msgs::Path global_path;
geometry_msgs::PoseStamped local_goal;
ros::Subscriber odom_sub;
ros::Subscriber gpath_sub;
ros::Subscriber map_sub;
ros::Publisher local_goal_pub;


int PathSize ;
double cur_rx;
double cur_ry;
double target = 0.3;
bool Path_Sub = false;
