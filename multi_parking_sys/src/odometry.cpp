#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>

#define WHEEL_RADIUS 0.033
#define WHEEL_BASE 0.33
#define PI 3.141592
#define TORPM 0.229
#define DT 1.0 / 100.0
ros::Publisher odom_pub;
tf2::Quaternion Quaternion_;

double x = 0.0;              // robot's x position in meters
double y = 0.0;              // robot's y position in meters
double theta = 0.0;          // robot's heading in radians
double left_velocity = 0.0;  // left wheel velocity in m/s
double right_velocity = 0.0; // right wheel velocity in m/s
double velocity = 0.0;
double omega = 0.0;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // ROS_INFO("Number of scan ranges: %lu", msg->ranges.size());
}

void msgCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    dynamixel_workbench_msgs::DynamixelState dynamixel_state[2];

    dynamixel_state[0].present_position = msg->dynamixel_state[0].present_position;
    dynamixel_state[0].present_velocity = msg->dynamixel_state[0].present_velocity;
    dynamixel_state[0].present_current = msg->dynamixel_state[0].present_current;

    dynamixel_state[1].present_position = msg->dynamixel_state[1].present_position;
    dynamixel_state[1].present_velocity = msg->dynamixel_state[1].present_velocity;
    dynamixel_state[1].present_current = msg->dynamixel_state[1].present_current;

    left_velocity = dynamixel_state[0].present_velocity * TORPM * (PI / 30.0) * WHEEL_RADIUS;
    right_velocity = dynamixel_state[1].present_velocity * TORPM * (PI / 30.0) * WHEEL_RADIUS;
}

int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "Odometry");
    ROS_INFO("--- odometry node ---");

    ros::NodeHandle nh;

    // param check
    std::string s;
    if (ros::param::get("~namespace", s))
        ROS_INFO("Odom node got param: %s", s.c_str());
    else
        ROS_ERROR("Failed to get param 'namespace'");

    // path setting
    std::string path = "/" + s + "/dynamixel_state";
    std::string odom_path = s + "/odom";
    std::string baselink_path = s + "/base_link";

    ROS_INFO("odom_path is : %s", odom_path.c_str());
    ROS_INFO("base_link path is : %s", baselink_path.c_str());

    //
    odom_pub = nh.advertise<nav_msgs::Odometry>(odom_path, 100);
    ros::Subscriber ros_dynamixel_sub = nh.subscribe(path, 100, msgCallback);
    ros::Rate loop_rate(100);
    tf::TransformBroadcaster odom_broadcaster;

    while (ros::ok())
    {
        velocity = (left_velocity + right_velocity) / 2.0;
        omega = (-left_velocity + right_velocity) / WHEEL_BASE;

        x += velocity * DT * cos(theta + (omega * DT / 2.0));
        y += velocity * DT * sin(theta + (omega * DT / 2.0));

        theta += omega * DT;

        Quaternion_.setRPY(0, 0, theta);
        Quaternion_ = Quaternion_.normalize();

        // About TF Transform
        ///////////////////////////////////////////////////////////////////////////////////
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odom_path;
        odom_trans.child_frame_id = baselink_path;

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        odom_trans.transform.rotation.x = Quaternion_.x();
        odom_trans.transform.rotation.y = Quaternion_.y();
        odom_trans.transform.rotation.z = Quaternion_.z();
        odom_trans.transform.rotation.w = Quaternion_.w();

        odom_broadcaster.sendTransform(odom_trans);
        ///////////////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////////////////////
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();

        odom_msg.header.frame_id = odom_path;
        odom_msg.child_frame_id = baselink_path;
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = Quaternion_.x();
        odom_msg.pose.pose.orientation.y = Quaternion_.y();
        odom_msg.pose.pose.orientation.z = Quaternion_.z();
        odom_msg.pose.pose.orientation.w = Quaternion_.w();

        odom_msg.twist.twist.linear.x = velocity;
        odom_msg.twist.twist.angular.z = omega;

        odom_pub.publish(odom_msg);
        ///////////////////////////////////////////////////////////////////////////////////

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
