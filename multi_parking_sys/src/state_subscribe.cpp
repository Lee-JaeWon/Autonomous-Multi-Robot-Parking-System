#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

dynamixel_workbench_msgs::DynamixelState dynamixel_list[2] = {};

void dynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    for (int index = 0; index < 2; index++)
    {
        dynamixel_list[index].present_current = msg->dynamixel_state[index].present_current;
        dynamixel_list[index].present_velocity = msg->dynamixel_state[index].present_velocity;
        dynamixel_list[index].present_position = msg->dynamixel_state[index].present_position;

        ROS_INFO("%d", msg->dynamixel_state[index].id);
        ROS_INFO("%d", msg->dynamixel_state[index].present_current);
        ROS_INFO("%d", msg->dynamixel_state[index].present_velocity);
        ROS_INFO("%d", msg->dynamixel_state[index].present_position);
        ROS_INFO("--------------------------------");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_state_sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/dynamixel_workbench_one/dynamixel_state", 1000, dynamixelStateCallback);

    ros::spin();

    return 0;
}