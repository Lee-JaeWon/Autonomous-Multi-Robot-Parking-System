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
geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::ConstPtr& odom_pose,const geometry_msgs::ConstPtr& init_pose);

// Create ROS node handle
int cnt = 1;

geometry_msgs::PoseStamped odom_pose;
geometry_msgs::PoseStamped odom_pose1;
geometry_msgs::PoseStamped odom_pose2;



void PoseCallback_0(const geometry_msgs::PoseStamped::ConstPtr& msg){
    odom_pose = *msg;
}

void PoseCallback_1(const geometry_msgs::PoseStamped::ConstPtr& msg){
    odom_pose1 = *msg;
}

void PoseCallback_2(const geometry_msgs::PoseStamped::ConstPtr& msg){
    odom_pose2 = *msg;
}


void initialPoseCallback_0(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // serviceCall(msg->pose.pose ,odom_pose.pose); // odom, init
 
    ROS_INFO("POSE_0 subscribed");

    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response startres;
    cartographer_ros_msgs::FinishTrajectory::Response finishres;

    // create Pose msg 
    geometry_msgs::Pose target_pose;

    finishreq.trajectory_id = cnt ; // int
    if (finishclient.call(finishreq, finishres)) ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua"; // string
    startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true; // bool


    // double odom_roll, odom_pitch, odom_yaw;
    // double init_roll, init_pitch, init_yaw;

    // //  odom_Quaternion to RPY
    // tf::Quaternion odom_quat(odom_pose.pose.orientation.x,odom_pose.pose.orientation.y,odom_pose.pose.orientation.z,odom_pose.pose.orientation.w)
    // tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);

    // // initialpose_Quaternion to RPY
    // tf::Quaternion init_quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w)
    // tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);
  
    // double target_roll,target_pitch,target_yaw;

    // target_pitch = init_pitch - odom_pitch ;
    // target_roll  = init_roll  - odom_roll  ;
    // target_yaw   = init_yaw   - odom_yaw ;

    // tf2::Quaternion target_quat;
    // // target_RPY to Quaternion
    // target_quat.setRPY( target_pitch, target_roll, target_yaw ); 

    // // msg->pose.pose
    // target_pose.position.x = msg->pose.pose.position.x - odom_pose.pose.position.x
    // target_pose.position.y = msg->pose.pose.position.y - odom_pose.pose.position.y
    // target_pose.position.z = 0 

    // target_pose.orientation.x = target_quat.getX();
    // target_pose.orientation.y = target_quat.getY();
    // target_pose.orientation.z = target_quat.getZ();
    // target_pose.orientation.w = target_quat.getW();


    // startreq.initial_pose = target_pose; // geometry _msgs
    startreq.initial_pose = Quat2RPY2Quat(odom_pose.pose, msg->pose.pose); // geometry _msgs

    startreq.relative_to_trajectory_id = cnt; /// int 
    // Call the start_trajectory service
    if(Startclient.call(startreq, startres)) ROS_INFO("start_trajectory service success..");

    cnt ++;


}


void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    serviceCall(msg->pose.pose ,odom_pose1.pose); // odom, init
}

void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    serviceCall(msg->pose.pose ,odom_pose2.pose); // odom, init
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

geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::ConstPtr& odom_msg,const geometry_msgs::ConstPtr& init_msg)
{
    // method that returns angle differences

    double odom_roll, odom_pitch, odom_yaw;
    double init_roll, init_pitch, init_yaw;

    geometry_msgs::Pose target_pose;

    //  odom_Quaternion to RPY
    tf::Quaternion odom_quat(odom_msg->orientation.x,odom_msg->orientation.y,odom_msg->orientation.z,odom_msg->orientation.w)
    tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);

    // initialpose_Quaternion to RPY
    tf::Quaternion init_quat(init_msg->orientation.x,init_msg->orientation.y,init_msg->orientation.z,init_msg->orientation.w)
    tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);
  
    double target_roll,target_pitch,target_yaw;

    target_roll  = init_roll  - odom_roll  ;
    target_pitch = init_pitch - odom_pitch ;
    target_yaw   = init_yaw   - odom_yaw ;

    tf2::Quaternion target_quat;
    // target_RPY to Quaternion
    // target_quat.setRPY( target_pitch, target_roll, target_yaw ); 
    target_quat.setRPY( 0, 0, target_yaw ); 

    target_quat = target_quat.normalize();


    // msg->pose.pose
    target_pose.position.x = init_msg->position.x - odom_msg->position.x;
    target_pose.position.y = init_msg->position.y - odom_msg->position.y;
    target_pose.position.z = 0 ;

    target_pose.orientation.x = target_quat.x();
    target_pose.orientation.y = target_quat.y();
    target_pose.orientation.z = target_quat.z();
    target_pose.orientation.w = target_quat.w();    

    return target_pose;
}

void serviceCall(const geometry_msgs::Pose::ConstPtr& odom_msg,const geometry_msgs::Pose::ConstPtr& init_msg)
{
    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response startres;
    cartographer_ros_msgs::FinishTrajectory::Response finishres;

    finishreq.trajectory_id = cnt ; // int
    if (finishclient.call(finishreq, finishres)) ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua"; // string
    startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true; // bool
    // startreq.initial_pose = target_pose; // geometry _msgs
    startreq.initial_pose = Quat2RPY2Quat(odom_msg, init_msg); // geometry _msgs

    startreq.relative_to_trajectory_id = cnt; /// int 
    // Call the start_trajectory service
    if(Startclient.call(startreq, startres)) ROS_INFO("start_trajectory service success..");

    cnt ++;


}