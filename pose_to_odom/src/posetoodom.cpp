#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/transform_listener.h>

std::string s1 = "";
ros::Publisher pub;
nav_msgs::Odometry odom;

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

    std::string pub_ns_one = "/" + s1 + "/odom_pto";
    pub = nh.advertise<nav_msgs::Odometry>(pub_ns_one, 33);

    // Subscribe to the pose topic
    // ros::Subscriber sub = nh.subscribe<geometry_msgs::PoseStamped>(s1 + "/tracked_pose", 10, poseStampedCallback);

    std::string base_ns = s1 + "/base_link";
    double robot_x = 0.0;
    double robot_y = 0.0;

    int hz = 33;
    ros::Rate rate(hz);
    tf::TransformListener listener;
    tf::StampedTransform transform;

    while (ros::ok())
    {
        try
        {
            listener.lookupTransform("map", base_ns,
                                     ros::Time(0), transform);
            robot_x = transform.getOrigin().x();
            robot_y = transform.getOrigin().y();
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        odom.header.frame_id = s1+"/odom_pto";
        odom.pose.pose.position.x = robot_x;
        odom.pose.pose.position.y = robot_y;


        pub.publish(odom);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
