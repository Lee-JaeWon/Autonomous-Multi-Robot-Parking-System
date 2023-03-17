// #include <ros/ros.h>
// #include <cartographer_ros_msgs/StartTrajectory.h>
// #include <cartographer_ros_msgs/FinishTrajectory.h>


// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "start_trajectory_client");
//   ros::NodeHandle n;

//   // Create a service client for the start_trajectory service
//   ros::ServiceClient Startclient = 
//     n.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
//   ros::ServiceClient finishclient = 
//     n.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");


//   // Create a StartTrajectoryRequest object and fill it with the necessary parameters
//   cartographer_ros_msgs::StartTrajectory::Request startreq;
//   cartographer_ros_msgs::FinishTrajectory::Request finishreq;
//   finishreq.trajectory_id = 1 ; // int
//   Startclient.call(finishreq, res);

//   startreq.configuration_basename = "my_map.lua"; // string
//   startreq.configuration_directory=""; // string
//   startreq.use_initial_pose = true; // bool
//   startreq.initial_pose = geometry_msgs; // geometry _msgs
//   startreq.relative_to_trajectory_id= ; /// int 
//   startreq.trajectory_id = // int
//   // Call the start_trajectory service
//   cartographer_ros_msgs::StartTrajectory::Response res;
//   Startclient.call(startreq, res)

//   return 0;
// }




#include <ros/ros.h>
#include <cartographer_ros_msgs/StartTrajectory.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

// Initialize ROS node


// Create ROS node handle
int cnt = 0;

geometry_msgs::PoseStamped Pose_msg;

void initialPoseCallback_0(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{

    ROS_INFO("POSE_0 subscribed");

    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response startres;
    cartographer_ros_msgs::FinishTrajectory::Response finishres;

    finishreq.trajectory_id = cnt ; // int
    if (finishclient.call(finishreq, finishres))
        ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua"; // string
    startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true; // bool

    geometry_msgs::Pose target_pose;

    double odom_roll, odom_pitch, odom_yaw;
    double init_roll, init_pitch, init_yaw;
    tf::Quaterinion odom_quat(Pose_msg.pose.orientation.x,Pose_msg.pose.orientation.y,Pose_msg.pose.orientation.z,Pose_msg.pose.orientation.w)
    tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);
    tf::Quaterinion init_quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,Pose_msg.pose.orientation.w)
    tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);
  
    double target_roll,target_pitch,target_yaw;

    target_pitch = odom_pitch - init_pitch;
    target_roll = odom_roll - init_roll;
    target_yaw = odom_yaw - init_yaw;

    tf2::Quaternion target_quat;
    target_quat.setRPY( target_pitch,target_roll,target_yaw ); 

    // msg->pose.pose
    target_pose.position.x = msg->pose.pose.position.x - Pose_msg.pose.position.x
    target_pose.position.y = msg->pose.pose.position.y - Pose_msg.pose.position.y
    target_pose.position.z = 0 

    target_pose.orientation.x = target_quat.getX();
    target_pose.orientation.y = target_quat.getY();
    target_pose.orientation.z = target_quat.getZ();
    target_pose.orientation.w = target_quat.getW();


    startreq.initial_pose = target_pose; // geometry _msgs


    startreq.relative_to_trajectory_id = cnt; /// int 
    // Call the start_trajectory service

    if(Startclient.call(startreq, startres))
        ROS_INFO("start_trajectory service success..");

    cnt ++;


}


void PoseCallback_0(const geometry_msgs::PoseStamped::ConstPtr& msg){
    Pose_msg = *msg;
}

void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("POSE_1 subscribed");
    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response res;
    cartographer_ros_msgs::StartTrajectory::Response finishres;

    finishreq.trajectory_id = cnt ; // int
    if (finishclient.call(finishreq, finishres))
        ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua"; // string
    startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true; // bool
    startreq.initial_pose = msg->pose.pose; // geometry _msgs
    startreq.relative_to_trajectory_id = cnt; /// int 
    // Call the start_trajectory service

    if(Startclient.call(startreq, res))
        ROS_INFO("start_trajectory service success..");

    cnt ++;
}

void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("POSE_2 subscribed");

    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response res;
    cartographer_ros_msgs::StartTrajectory::Response finishres;

    finishreq.trajectory_id = cnt ; // int
    if (finishclient.call(finishreq, finishres))
        ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua"; // string
    startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true; // bool
    startreq.initial_pose = msg->pose.pose; // geometry _msgs
    startreq.relative_to_trajectory_id = cnt; /// int 
    // Call the start_trajectory service

    if(Startclient.call(startreq, res))
        ROS_INFO("start_trajectory service success..");

    cnt ++;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"initial_pose_subscriber");
    // Create subscriber to initial pose topic
    ros::NodeHandle nh;

    ros::Subscriber initialPoseSub_0 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_0", 1, initialPoseCallback_0);
    ros::Subscriber initialPoseSub_1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_1", 1, initialPoseCallback_1);
    ros::Subscriber initialPoseSub_2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_2", 1, initialPoseCallback_2);

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 1, PoseCallback_0);




    // Spin and process callbacks
    ros::spin();

    return 0;
}