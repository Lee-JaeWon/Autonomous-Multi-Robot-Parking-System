#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include <string>

dynamixel_workbench_msgs::DynamixelState dynamixel_list[2] = {};

void dynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    // ROS_INFO("%s\n", name.c_str());
    for (int index = 0; index < 2; index++)
    {
        dynamixel_list[index].present_current = msg->dynamixel_state[index].present_current;
        dynamixel_list[index].present_velocity = msg->dynamixel_state[index].present_velocity;
        dynamixel_list[index].present_position = msg->dynamixel_state[index].present_position;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_state_sub");
    ros::NodeHandle nh;

    std::string s;
    if (ros::param::get("~namespace", s))
    {
        ROS_INFO("Got param: %s", s.c_str());
    }
    else
    {
        ROS_ERROR("Failed to get param 'namespace'");
    }

    std::string path = "/" + s + "/dynamixel_state";

    ros::Subscriber sub = nh.subscribe(path, 1000, dynamixelStateCallback);

    ros::spin();

    return 0;
}