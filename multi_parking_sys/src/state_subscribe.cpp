#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include <string>

dynamixel_workbench_msgs::DynamixelState dynamixel_list[2] = {};

std::string name_robot;

void dynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    std::cout << "lalal" << "\n";
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

    nh.getParam("namespace", name_robot);

    ros::Subscriber sub = nh.subscribe("/robot_2/dynamixel_state", 1000, dynamixelStateCallback);

    ros::spin();

    return 0;
}