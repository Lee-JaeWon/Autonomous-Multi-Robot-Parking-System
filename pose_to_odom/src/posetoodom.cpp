#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

std::string s1 = "";
ros::Publisher pub;
nav_msgs::Odometry odom;

void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(s1 != ""){
        odom.header.seq = msg->header.seq;
        odom.header.stamp = msg->header.stamp;
        odom.header.frame_id = s1;

        odom.pose.pose.position = msg->pose.position;
        odom.pose.pose.orientation = msg->pose.orientation;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_to_odom");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    if (ros::param::get("~namespace", s1))
    {
        ROS_INFO("pose_to_odom node got param: %s", s1.c_str());
    }
    else
        ROS_ERROR("pose_to_odom node Failed to get param '%s'", s1.c_str());

    std::string pub_ns_one = "/" + s1 + "/pto";
    pub = nh.advertise<nav_msgs::Odometry>(pub_ns_one, 1000);

    // Subscribe to the pose topic
    ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(s1 + "/tracked_pose", 10, poseStampedCallback);

    int hz = 1000;
    ros::Rate rate(hz);
    while (ros::ok())
    {
        pub.publish(odom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
