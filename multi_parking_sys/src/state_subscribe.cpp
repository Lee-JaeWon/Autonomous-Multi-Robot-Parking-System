#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

void dynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg)
{
    for (int index = 0; index < 2; index++)
    {
        ROS_INFO("%d", msg->dynamixel_state[index].present_current);
        ROS_INFO("%d", msg->dynamixel_state[index].present_velocity);
        ROS_INFO("%d", msg->dynamixel_state[index].present_position);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_sub");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/dynamixel_workbench_one/dynamixel_state", 1000, dynamixelStateCallback);

    ros::spin();

    return 0;
}