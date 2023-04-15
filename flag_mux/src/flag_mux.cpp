#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>

bool nsone_velstop_flag = false;

void nsone_cmd_velstop_Callback(const std_msgs::Bool::ConstPtr &msg)
{
    nsone_velstop_flag = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flag_mux_node");

    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    // namespace
    std::string ns_one = "/robot_1";
    //

    // Subscriber
    ros::Subscriber nsone_cmd_vel_stop_sub;
    nsone_cmd_vel_stop_sub = nh.subscribe(ns_one + "/cmd_vel_stop_tomux", 1000, nsone_cmd_velstop_Callback); // from ui_main
    //

    // Publisher
    ros::Publisher nsone_cmd_vel_stop_pub;
    nsone_cmd_vel_stop_pub = nh.advertise<std_msgs::Bool>(ns_one + "/cmd_vel_stop", 1000);
    //

    while (ros::ok())
    {
        ros::spinOnce();   // callback 호출
        loop_rate.sleep(); // 루프 주기

        if (nsone_velstop_flag) // ui_main/qnode.cpp -to- mux -to- tracker.cpp // is stop tracker.cpp sign.
        {
            ROS_INFO_ONCE("nsone_velstop_flag activate::tracker.cpp stop");
            std_msgs::Bool bool_msg;
            bool_msg.data = false;
            nsone_cmd_vel_stop_pub.publish(bool_msg); // tracker stop sign
        }
        else
        {
            std_msgs::Bool bool_msg;
            bool_msg.data = true;
            nsone_cmd_vel_stop_pub.publish(bool_msg); // tracker start sign
        }


        // ROS Basic check
        if (!ros::master::check())
        {
            ROS_ERROR("ROS master disconnected");
            break;
        }
    }

    return 0;
}