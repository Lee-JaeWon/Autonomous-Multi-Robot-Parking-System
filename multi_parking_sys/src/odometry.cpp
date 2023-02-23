#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "sensor_msgs/LaserScan.h"

#define WHEEL_RADIUS 0.033
#define WHEEL_BASE 0.361
#define PI 3.141592
#define TORPM 0.229
#define DT 1.0 /100.0
ros::Publisher odom_pub;
nav_msgs::Odometry odom_msg;
tf2::Quaternion Quaternion_;

double x = 0.0;// robot's x position in meters
double y = 0.0;// robot's y position in meters
double theta = 0.0;// robot's heading in radians
double left_velocity = 0.0;// left wheel velocity in m/s
double right_velocity = 0.0;// right wheel velocity in m/s
double velocity= 0.0;
double omega= 0.0;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("Number of scan ranges: %lu", msg->ranges.size());
}

void msgCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
{

    dynamixel_workbench_msgs::DynamixelState dynamixel_state[2];

    dynamixel_state[0].present_position = msg -> dynamixel_state[0].present_position;
    dynamixel_state[0].present_velocity = msg -> dynamixel_state[0].present_velocity;
    dynamixel_state[0].present_current = msg -> dynamixel_state[0].present_current;

    dynamixel_state[1].present_position = msg->dynamixel_state[1].present_position;
    dynamixel_state[1].present_velocity = msg->dynamixel_state[1].present_velocity;
    dynamixel_state[1].present_current = msg->dynamixel_state[1].present_current;

    // ROS_INFO("%d",dynamixel_state[0].present_current );
    // ROS_INFO("%d",dynamixel_state[index].present_velocity );
    // ROS_INFO("%d",dynamixel_state[index].present_position );

    left_velocity = dynamixel_state[0].present_velocity * TORPM   * (PI / 30.0) * WHEEL_RADIUS;
    right_velocity = dynamixel_state[1].present_velocity * TORPM  * (PI / 30.0) * WHEEL_RADIUS;
    std::cout << "Dynamixel OK"<<std::endl;

}

int main(int argc, char **argv)
{


    ros::init(argc, argv, "Odometry");
    ros::NodeHandle nh;
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    ros::Subscriber ros_dynamixel_sub = nh.subscribe("/dynamixel_workbench_one/dynamixel_state", 100, msgCallback);
    ros::Rate loop_rate(100);

    while (ros::ok())
    {   
        velocity = (left_velocity + right_velocity) / 2.0;
        omega    = (left_velocity - right_velocity) / WHEEL_BASE;
        
        x += velocity * DT * cos(theta + (omega * DT / 2.0));
        y += velocity * DT * sin(theta + (omega * DT / 2.0));
        
        theta += omega * DT;

        Quaternion_.setRPY(0,0,theta);
        Quaternion_ = Quaternion_.normalize();

        odom_msg.header.stamp = ros::Time::now();

        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
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

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

    
}
