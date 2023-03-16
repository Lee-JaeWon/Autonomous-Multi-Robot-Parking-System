#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void initialPoseCallback_0(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Initial pose received, do something with it
    ROS_INFO("Initial position received: x=%f\n",msg->pose.pose.position.x);
    ROS_INFO("Initial position received: y=%f\n",msg->pose.pose.position.y);
    ROS_INFO("Initial position received: z=%f\n",msg->pose.pose.position.z);
    ROS_INFO("Initial orientation received: x=%f\n",msg->pose.pose.orientation.x);
    ROS_INFO("Initial orientation received: y=%f\n",msg->pose.pose.orientation.y);
    ROS_INFO("Initial orientation received: z=%f\n",msg->pose.pose.orientation.z);
    ROS_INFO("Initial orientation received: w=%f\n",msg->pose.pose.orientation.w);
}
void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Initial pose received, do something with it
    ROS_INFO("Initial position received: x=%f\n",msg->pose.pose.position.x);
    ROS_INFO("Initial position received: y=%f\n",msg->pose.pose.position.y);
    ROS_INFO("Initial position received: z=%f\n",msg->pose.pose.position.z);
    ROS_INFO("Initial orientation received: x=%f\n",msg->pose.pose.orientation.x);
    ROS_INFO("Initial orientation received: y=%f\n",msg->pose.pose.orientation.y);
    ROS_INFO("Initial orientation received: z=%f\n",msg->pose.pose.orientation.z);
    ROS_INFO("Initial orientation received: w=%f\n",msg->pose.pose.orientation.w);
}
void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // Initial pose received, do something with it
    ROS_INFO("Initial position received: x=%f\n",msg->pose.pose.position.x);
    ROS_INFO("Initial position received: y=%f\n",msg->pose.pose.position.y);
    ROS_INFO("Initial position received: z=%f\n",msg->pose.pose.position.z);
    ROS_INFO("Initial orientation received: x=%f\n",msg->pose.pose.orientation.x);
    ROS_INFO("Initial orientation received: y=%f\n",msg->pose.pose.orientation.y);
    ROS_INFO("Initial orientation received: z=%f\n",msg->pose.pose.orientation.z);
    ROS_INFO("Initial orientation received: w=%f\n",msg->pose.pose.orientation.w);
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "initial_pose_subscriber");

    // Create ROS node handle
    ros::NodeHandle nh;

    // Create subscriber to initial pose topic
    ros::Subscriber initialPoseSub_0 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_0", 1, initialPoseCallback_0);
    ros::Subscriber initialPoseSub_1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_1", 1, initialPoseCallback_1);
    ros::Subscriber initialPoseSub_2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_2", 1, initialPoseCallback_2);

    // Spin and process callbacks
    ros::spin();

    return 0;
}
