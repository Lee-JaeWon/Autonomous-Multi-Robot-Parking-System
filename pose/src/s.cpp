// #include <ros/ros.h>
// #include <cartographer_ros_msgs/StartTrajectory.h>
// #include <cartographer_ros_msgs/FinishTrajectory.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Quaternion.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf/tf.h>

// // Initialize ROS node

// // Create ROS node handle
// int cnt = 1;

// // namespace param
// std::string s1;
// std::string s2;
// std::string s3;

// geometry_msgs::PoseStamped Pose_msg;

// void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// 2
// 3   ROS_INFO("robot_1 pose is subscribed.");

//     // Create a service client for the start_trajectory service
//     ros::NodeHandle nh;

//     std::string st1 = "/robot_1/start_trajectory";
//     std::string ft1 = "/robot_1/finish_trajectory";

//     ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>(st1);
//     ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>(ft1);

//     // Create a StartTrajectoryRequest object and fill it with the necessary parameters
//     cartographer_ros_msgs::StartTrajectory::Request startreq;
//     cartographer_ros_msgs::FinishTrajectory::Request finishreq;

//     cartographer_ros_msgs::StartTrajectory::Response startres;
//     cartographer_ros_msgs::FinishTrajectory::Response finishres;

//     finishreq.trajectory_id = cnt; // int
//     if (finishclient.call(finishreq, finishres))
//         ROS_INFO("finish_trajectory service success..");

//     std::string lua_path1 = s1 + "_rplidar.lua";
//     startreq.configuration_basename = lua_path1;                                                                                     // stringss
//     startreq.configuration_directory = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
//     startreq.use_initial_pose = true;                                                                                                // bool

//     double odom_roll, odom_pitch, odom_yaw;
//     double init_roll, init_pitch, init_yaw;
//     tf::Quaternion odom_quat(Pose_msg.pose.orientation.x, Pose_msg.pose.orientation.y, Pose_msg.pose.orientation.z, Pose_msg.pose.orientation.w);
//     tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);
//     tf::Quaternion init_quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, Pose_msg.pose.orientation.w);
//     tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);

//     double target_roll, target_pitch, target_yaw;

//     target_pitch = init_pitch - odom_pitch;
//     target_roll = init_roll - odom_roll;
//     target_yaw = init_yaw - odom_yaw;

//     tf2::Quaternion target_quat;
//     target_quat.setRPY(target_pitch, target_roll, target_yaw);

//     target_quat = target_quat.normalize();

//     // msg->pose.pose
//     geometry_msgs::Pose target_pose;

//     target_pose.position.x = (msg->pose.pose.position.x - Pose_msg.pose.position.x);
//     target_pose.position.y = (msg->pose.pose.position.y - Pose_msg.pose.position.y);
//     target_pose.position.z = 0;

//     target_pose.orientation.x = target_quat.getX();
//     target_pose.orientation.y = target_quat.getY();
//     target_pose.orientation.z = target_quat.getZ();
//     target_pose.orientation.w = target_quat.getW();

//     startreq.initial_pose = target_pose; // geometry _msgs

//     startreq.relative_to_trajectory_id = cnt; /// int
//     // Call the start_trajectory service

//     if (Startclient.call(startreq, startres))
//         ROS_INFO("start_trajectory service success..");

//     cnt++;
// }

// void PoseCallback_0(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     Pose_msg = *msg;
// }

// void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//     ROS_INFO("POSE_1 subscribed");
//     // Create a service client for the start_trajectory service
//     ros::NodeHandle nh;
//     ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
//     ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

//     // Create a StartTrajectoryRequest object and fill it with the necessary parameters
//     cartographer_ros_msgs::StartTrajectory::Request startreq;
//     cartographer_ros_msgs::FinishTrajectory::Request finishreq;

//     cartographer_ros_msgs::StartTrajectory::Response res;
//     cartographer_ros_msgs::StartTrajectory::Response finishres;

//     finishreq.trajectory_id = cnt; // int
//     if (finishclient.call(finishreq, finishres))
//         ROS_INFO("finish_trajectory service success..");

//     std::string lua_path2 = s2 + "_rplidar.lua";
//     startreq.configuration_basename = lua_path2;                                                                                     // string
//     startreq.configuration_directory = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
//     startreq.use_initial_pose = true;                                                                                                // bool
//     startreq.initial_pose = msg->pose.pose;                                                                                          // geometry _msgs
//     startreq.relative_to_trajectory_id = cnt;                                                                                        /// int
//     // Call the start_trajectory service

//     if (Startclient.call(startreq, res))
//         ROS_INFO("start_trajectory service success..");

//     cnt++;
// }

// void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
// {
//     ROS_INFO("POSE_2 subscribed");

//     // Create a service client for the start_trajectory service
//     ros::NodeHandle nh;
//     ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
//     ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

//     // Create a StartTrajectoryRequest object and fill it with the necessary parameters
//     cartographer_ros_msgs::StartTrajectory::Request startreq;
//     cartographer_ros_msgs::FinishTrajectory::Request finishreq;

//     cartographer_ros_msgs::StartTrajectory::Response res;
//     cartographer_ros_msgs::StartTrajectory::Response finishres;

//     finishreq.trajectory_id = cnt; // int
//     if (finishclient.call(finishreq, finishres))
//         ROS_INFO("finish_trajectory service success..");

//     std::string lua_path3 = s3 + "_rplidar.lua";
//     startreq.configuration_basename = lua_path3;                                                                                     // string
//     startreq.configuration_directory = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
//     startreq.use_initial_pose = true;                                                                                                // bool
//     startreq.initial_pose = msg->pose.pose;                                                                                          // geometry _msgs
//     startreq.relative_to_trajectory_id = cnt;                                                                                        /// int
//     // Call the start_trajectory service

//     if (Startclient.call(startreq, res))
//         ROS_INFO("start_trajectory service success..");

//     cnt++;
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "initial_pose_subscriber");
//     // Create subscriber to initial pose topic
//     ros::NodeHandle nh;

//     // param check
//     if (ros::param::get("~namespace_one", s1))
//         ROS_INFO("Init_pose node Got 'namespace_one' param: %s", s1.c_str());
//     else
//         ROS_ERROR("Failed to get param 'namespace_one'");
//     if (ros::param::get("~namespace_two", s2))
//         ROS_INFO("Init_pose node Got 'namespace_two' param: %s", s2.c_str());
//     else
//         ROS_ERROR("Failed to get param 'namespace_two'");
//     if (ros::param::get("~namespace_three", s3))
//         ROS_INFO("Init_pose node Got 'namespace_three' param: %s", s3.c_str());
//     else
//         ROS_ERROR("Failed to get param 'namespace_three'");

//     std::string init_pose_one = '/' + s1 + "/initialpose";
//     std::string init_pose_two = '/' + s2 + "/initialpose";
//     std::string init_pose_three = '/' + s3 + "/initialpose";

//     ros::Subscriber initialPoseSub_0 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(init_pose_one, 1, initialPoseCallback_1);
//     ros::Subscriber initialPoseSub_1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(init_pose_two, 1, initialPoseCallback_2);
//     ros::Subscriber initialPoseSub_2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(init_pose_three, 1, initialPoseCallbac3_2);

//     ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 1, PoseCallback_0);

//     // Spin and process callbacks
//     ros::spin();

//     return 0;
// }

// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // #include <ros/ros.h>
// // #include <cartographer_ros_msgs/StartTrajectory.h>
// // #include <cartographer_ros_msgs/FinishTrajectory.h>

// // int main(int argc, char **argv)
// // {
// //   ros::init(argc, argv, "start_trajectory_client");
// //   ros::NodeHandle n;

// //   // Create a service client for the start_trajectory service
// //   ros::ServiceClient Startclient =
// //     n.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
// //   ros::ServiceClient finishclient =
// //     n.serviceClient<cartographer_ros_msgs::StartTrajectory>("/finish_trajectory");

// //   // Create a StartTrajectoryRequest object and fill it with the necessary parameters
// //   cartographer_ros_msgs::StartTrajectory::Request startreq;
// //   cartographer_ros_msgs::FinishTrajectory::Request finishreq;
// //   finishreq.trajectory_id = 1 ; // int
// //   Startclient.call(finishreq, res);

// //   startreq.configuration_basename = "my_map.lua"; // string
// //   startreq.configuration_directory=""; // string
// //   startreq.use_initial_pose = true; // bool
// //   startreq.initial_pose = geometry_msgs; // geometry _msgs
// //   startreq.relative_to_trajectory_id= ; /// int
// //   startreq.trajectory_id = // int
// //   // Call the start_trajectory service
// //   cartographer_ros_msgs::StartTrajectory::Response res;
// //   Startclient.call(startreq, res)

// //   return 0;
// // }

// // #include <ros/ros.h>
// // #include <cartographer_ros_msgs/StartTrajectory.h>
// // #include <cartographer_ros_msgs/FinishTrajectory.h>
// // #include <geometry_msgs/PoseWithCovarianceStamped.h>
// // #include <geometry_msgs/PoseStamped.h>
// // #include <geometry_msgs/Pose.h>
// // #include <geometry_msgs/Quaternion.h>
// // #include <tf2/LinearMath/Quaternion.h>
// // #include <tf/tf.h>

// // // Initialize ROS node
// // geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::Pose::ConstPtr& odom_pose1,const geometry_msgs::Pose::ConstPtr& init_pose);
// // void serviceCall(const geometry_msgs::Pose odom_msg,const geometry_msgs::Pose  init_msg);
// // // Create ROS node handle
// // int cnt = 1;

// // geometry_msgs::PoseStamped odom_pose1;
// // geometry_msgs::PoseStamped odom_pose2;
// // geometry_msgs::PoseStamped odom_pose3;

// // void PoseCallback_0(const geometry_msgs::PoseStamped::ConstPtr& msg){
// //     odom_pose1 = *msg;
// // }

// // void PoseCallback_1(const geometry_msgs::PoseStamped::ConstPtr& msg){
// //     odom_pose2 = *msg;
// // }

// // void PoseCallback_2(const geometry_msgs::PoseStamped::ConstPtr& msg){
// //     odom_pose3 = *msg;
// // }

// // void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped msg)
// // 2
// // 3   // serviceCall(msg->pose.pose ,odom_pose1.pose); // odom, init

// //     ROS_INFO("POSE_0 subscribed");

// //     // Create a service client for the start_trajectory service
// //     ros::NodeHandle nh;
// //     ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/robot_1/start_trajectory");
// //     ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/robot_1/finish_trajectory");

// //     // Create a StartTrajectoryRequest object and fill it with the necessary parameters
// //     cartographer_ros_msgs::StartTrajectory::Request startreq;
// //     cartographer_ros_msgs::FinishTrajectory::Request finishreq;

// //     cartographer_ros_msgs::StartTrajectory::Response startres;
// //     cartographer_ros_msgs::FinishTrajectory::Response finishres;

// //     // create Pose msg
// //     geometry_msgs::Pose target_pose;

// //     finishreq.trajectory_id = cnt ; // int
// //     if (finishclient.call(finishreq, finishres)) ROS_INFO("finish_trajectory service success..");

// //     startreq.configuration_basename = "robot_1_rplidar.lua"; // string
// //     startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
// //     startreq.use_initial_pose = true; // bool

// //     // double odom_roll, odom_pitch, odom_yaw;
// //     // double init_roll, init_pitch, init_yaw;

// //     // //  odom_Quaternion to RPY
// //     // tf::Quaternion odom_quat(odom_pose1.pose.orientation.x,odom_pose1.pose.orientation.y,odom_pose1.pose.orientation.z,odom_pose1.pose.orientation.w)
// //     // tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);

// //     // // initialpose_Quaternion to RPY
// //     // tf::Quaternion init_quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w)
// //     // tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);

// //     // double target_roll,target_pitch,target_yaw;

// //     // target_pitch = init_pitch - odom_pitch ;
// //     // target_roll  = init_roll  - odom_roll  ;
// //     // target_yaw   = init_yaw   - odom_yaw ;

// //     // tf2::Quaternion target_quat;
// //     // // target_RPY to Quaternion
// //     // target_quat.setRPY( target_pitch, target_roll, target_yaw );

// //     // // msg->pose.pose
// //     // target_pose.position.x = msg->pose.pose.position.x - odom_pose1.pose.position.x
// //     // target_pose.position.y = msg->pose.pose.position.y - odom_pose1.pose.position.y
// //     // target_pose.position.z = 0

// //     // target_pose.orientation.x = target_quat.getX();
// //     // target_pose.orientation.y = target_quat.getY();
// //     // target_pose.orientation.z = target_quat.getZ();
// //     // target_pose.orientation.w = target_quat.getW();

// //     // startreq.initial_pose = target_pose; // geometry _msgs
// //     startreq.initial_pose = Quat2RPY2Quat(odom_pose1.pose, msg.pose.pose); // geometry _msgs

// //     startreq.relative_to_trajectory_id = cnt; /// int
// //     // Call the start_trajectory service
// //     if(Startclient.call(startreq, startres)) ROS_INFO("start_trajectory service success..");

// //     cnt ++;

// // }

// // void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// // {
// //     serviceCall(msg->pose.pose ,odom_pose2.pose); // odom, init
// // }

// // void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// // {
// //     serviceCall(msg->pose.pose ,odom_pose3.pose); // odom, init
// // }

// // int main(int argc, char** argv)
// // {
// //     ros::init(argc, argv,"initial_pose_subscriber");
// //     // Create subscriber to initial pose topic
// //     ros::NodeHandle nh;
// //     ros::Subscriber initialPoseSub_0 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_0", 1, initialPoseCallback_1);
// //     ros::Subscriber initialPoseSub_1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_1", 1, initialPoseCallback_2);
// //     ros::Subscriber initialPoseSub_2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_2", 1, initialPoseCallback_3);

// //     ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 1, PoseCallback_0);

// //     // Spin and process callbacks
// //     ros::spin();

// //     return 0;
// // }

// // geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::Pose odom_msg,const geometry_msgs::Pose init_msg)
// // {
// //     // method that returns angle differences

// //     double odom_roll, odom_pitch, odom_yaw;
// //     double init_roll, init_pitch, init_yaw;

// //     geometry_msgs::Pose target_pose;

// //     //  odom_Quaternion to RPY
// //     tf::Quaternion odom_quat(odom_msg.orientation.x,odom_msg.orientation.y,odom_msg.orientation.z,odom_msg.orientation.w);
// //     tf::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);

// //     // initialpose_Quaternion to RPY
// //     tf::Quaternion init_quat(init_msg.orientation.x,init_msg.orientation.y,init_msg.orientation.z,init_msg.orientation.w);
// //     tf::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);

// //     double target_roll,target_pitch,target_yaw;

// //     target_roll  = init_roll  - odom_roll  ;
// //     target_pitch = init_pitch - odom_pitch ;
// //     target_yaw   = init_yaw   - odom_yaw ;

// //     tf2::Quaternion target_quat;
// //     // target_RPY to Quaternion
// //     // target_quat.setRPY( target_pitch, target_roll, target_yaw );
// //     target_quat.setRPY( 0, 0, target_yaw );

// //     target_quat = target_quat.normalize();

// //     // msg.pose.pose
// //     target_pose.position.x = init_msg.position.x - odom_msg.position.x;
// //     target_pose.position.y = init_msg.position.y - odom_msg.position.y;
// //     target_pose.position.z = 0 ;

// //     target_pose.orientation.x = target_quat.x();
// //     target_pose.orientation.y = target_quat.y();
// //     target_pose.orientation.z = target_quat.z();
// //     target_pose.orientation.w = target_quat.w();

// //     return target_pose;
// // }

// // void serviceCall(const geometry_msgs::Pose odom_msg,const geometry_msgs::Pose  init_msg)
// // {
// //     // Create a service client for the start_trajectory service
// //     ros::NodeHandle nh;
// //     ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>("/start_trajectory");
// //     ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>("/finish_trajectory");

// //     // Create a StartTrajectoryRequest object and fill it with the necessary parameters
// //     cartographer_ros_msgs::StartTrajectory::Request startreq;
// //     cartographer_ros_msgs::FinishTrajectory::Request finishreq;

// //     cartographer_ros_msgs::StartTrajectory::Response startres;
// //     cartographer_ros_msgs::FinishTrajectory::Response finishres;

// //     finishreq.trajectory_id = cnt ; // int
// //     if (finishclient.call(finishreq, finishres)) ROS_INFO("finish_trajectory service success..");

// //     startreq.configuration_basename = "rplidar.lua"; // string
// //     startreq.configuration_directory="/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
// //     startreq.use_initial_pose = true; // bool
// //     // startreq.initial_pose = target_pose; // geometry _msgs
// //     startreq.initial_pose = Quat2RPY2Quat(odom_msg, init_msg); // geometry _msgs

// //     startreq.relative_to_trajectory_id = cnt; /// int
// //     // Call the start_trajectory service
// //     if(Startclient.call(startreq, startres)) ROS_INFO("start_trajectory service success..");

// //     cnt ++;

// // }

#include <ros/ros.h>
#include <cartographer_ros_msgs/StartTrajectory.h>
#include <cartographer_ros_msgs/FinishTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Initialize ROS node
void serviceCall(const geometry_msgs::Pose &odom_msg, const geometry_msgs::Pose &init_msg);
geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::Pose &odom_msg, const geometry_msgs::Pose &init_msg);
// Create ROS node handle
int cnt = 1;

geometry_msgs::PoseStamped odom_pose1;
geometry_msgs::PoseStamped odom_pose2;
geometry_msgs::PoseStamped odom_pose3;

std::string s1 = "robot_1";
std::string s2 = "robot_2";
std::string s3 = "robot_3";

void PoseCallback_1(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odom_pose1 = *msg;
}

void PoseCallback_2(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odom_pose2 = *msg;
}

void PoseCallback_3(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    odom_pose3 = *msg;
}

void initialPoseCallback_1(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg){

   // serviceCall(msg.pose.pose ,odom_pose1.pose); // odom, init

    ROS_INFO("POSE_0 subscribed");

    // namespace
    std::string ns_st = "/" + s1 + "/start_trajectory";
    std::string ns_ft = "/" + s1 + "/finish_trajectory";

    // Create a service client for the start_trajectory service
    ros::NodeHandle nh;
    ros::ServiceClient Startclient = nh.serviceClient<cartographer_ros_msgs::StartTrajectory>(ns_st);
    ros::ServiceClient finishclient = nh.serviceClient<cartographer_ros_msgs::FinishTrajectory>(ns_ft);

    // Create a StartTrajectoryRequest object and fill it with the necessary parameters
    cartographer_ros_msgs::StartTrajectory::Request startreq;
    cartographer_ros_msgs::FinishTrajectory::Request finishreq;

    cartographer_ros_msgs::StartTrajectory::Response startres;
    cartographer_ros_msgs::FinishTrajectory::Response finishres;

    // create Pose msg
    geometry_msgs::Pose target_pose;

    finishreq.trajectory_id = cnt; // int
    if (finishclient.call(finishreq, finishres))
        ROS_INFO("finish_trajectory service success..");

    std::string ns_lua_1 = s1 + "_rplidar.lua";
    
    startreq.configuration_basename = ns_lua_1;                                                                                 // string
    startreq.configuration_directory = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true;                                                                                                // bool

    // double odom_roll, odom_pitch, odom_yaw;
    // double init_roll, init_pitch, init_yaw;

    // //  odom_Quaternion to RPY
    // tf::Quaternion odom_quat(odom_pose1.pose.orientation.x,odom_pose1.pose.orientation.y,odom_pose1.pose.orientation.z,odom_pose1.pose.orientation.w)
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
    // target_pose.position.x = msg->pose.pose.position.x - odom_pose1.pose.position.x
    // target_pose.position.y = msg->pose.pose.position.y - odom_pose1.pose.position.y
    // target_pose.position.z = 0

    // target_pose.orientation.x = target_quat.getX();
    // target_pose.orientation.y = target_quat.getY();
    // target_pose.orientation.z = target_quat.getZ();
    // target_pose.orientation.w = target_quat.getW();
    // startreq.initial_pose = target_pose; // geometry _msgs

    startreq.initial_pose = Quat2RPY2Quat(odom_pose1.pose, msg->pose.pose); // geometry_msgs/Pose data type

    startreq.relative_to_trajectory_id = cnt; /// int
    // Call the start_trajectory service
    if (Startclient.call(startreq, startres))
        ROS_INFO("start_trajectory service success..");

    cnt++;
}

void initialPoseCallback_2(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    geometry_msgs::Pose odom = msg->pose.pose;
    geometry_msgs::Pose init = odom_pose2.pose;
    serviceCall(odom, init); // odom, init
}

void initialPoseCallback_3(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

    geometry_msgs::Pose odom = msg->pose.pose;
    geometry_msgs::Pose init = odom_pose3.pose;
    serviceCall(odom, init); // odom, init
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "initial_pose_subscriber");
    ROS_INFO("initial_pose_subscriber");

    // param check
    if (ros::param::get("~namespace_one", s1))
        ROS_INFO("Init_Pose node got param: %s", s1.c_str());
    else
        ROS_ERROR("Failed to get param 'namespace_one'");

    
    if (ros::param::get("~namespace_two", s2))
        ROS_INFO("Init_Pose node got param: %s", s2.c_str());
    else
        ROS_ERROR("Failed to get param 'namespace_two'");

    
    if (ros::param::get("~namespace_three", s3))
        ROS_INFO("Init_Pose node got param: %s", s3.c_str());
    else
        ROS_ERROR("Failed to get param 'namespace_three'");

    // Create namespace
    std::string Init_ns_1 = "/" + s1 + "/initialpose";
    std::string Init_ns_2 = "/" + s2 + "/initialpose";
    std::string Init_ns_3 = "/" + s3 + "/initialpose";

    std::string pose_ns_1 = "/" + s1 + "/tracked_pose";
    std::string pose_ns_2 = "/" + s2 + "/tracked_pose";
    std::string pose_ns_3 = "/" + s3 + "/tracked_pose";



    // Create subscriber to initial pose topic
    ros::NodeHandle nh;
    ros::Subscriber robot_1_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_ns_1, 1, PoseCallback_1);
    ros::Subscriber robot_2_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_ns_2, 1, PoseCallback_2);
    ros::Subscriber robot_3_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(pose_ns_3, 1, PoseCallback_3);
    
    ros::Subscriber initialPoseSub_1 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(Init_ns_1, 1, initialPoseCallback_1);
    ros::Subscriber initialPoseSub_2 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(Init_ns_2, 1, initialPoseCallback_2);
    ros::Subscriber initialPoseSub_3 = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(Init_ns_3, 1, initialPoseCallback_3);



    // Spin and process callbacks
    ros::spin();

    return 0;
}

geometry_msgs::Pose Quat2RPY2Quat(const geometry_msgs::Pose &odom_msg, const geometry_msgs::Pose &init_msg)
{
    // method that returns angle differences

    double odom_roll, odom_pitch, odom_yaw;
    double init_roll, init_pitch, init_yaw;

    geometry_msgs::Pose target_pose;

    //  odom_Quaternion to RPY
    tf2::Quaternion odom_quat(odom_msg.orientation.x, odom_msg.orientation.y, odom_msg.orientation.z, odom_msg.orientation.w);
    tf2::Matrix3x3(odom_quat).getRPY(odom_roll, odom_pitch, odom_yaw);

    // initialpose_Quaternion to RPY
    tf2::Quaternion init_quat(init_msg.orientation.x, init_msg.orientation.y, init_msg.orientation.z, init_msg.orientation.w);
    tf2::Matrix3x3(init_quat).getRPY(init_roll, init_pitch, init_yaw);

    double target_roll, target_pitch, target_yaw;

    // ROS_INFO("%f %f", init_yaw, odom_yaw);

    target_roll = init_roll - odom_roll;
    target_pitch = init_pitch - odom_pitch;
    target_yaw = init_yaw - odom_yaw;

    tf2::Quaternion target_quat;
    // target_RPY to Quaternion
    // target_quat.setRPY( target_pitch, target_roll, target_yaw );
    target_quat.setRPY(0, 0, target_yaw);
    // ROS_INFO("%f", target_yaw);
    target_quat = target_quat.normalize();

    // ROS_INFO("%f %f %f %f",target_quat.x(),target_quat.y(),target_quat.z(),target_quat.w());

    // msg.pose.pose
    target_pose.position.x = init_msg.position.x - odom_msg.position.x;
    target_pose.position.y = init_msg.position.y - odom_msg.position.y;
    target_pose.position.z = 0;

    target_pose.orientation.x = target_quat.x();
    target_pose.orientation.y = target_quat.y();
    target_pose.orientation.z = target_quat.z();
    target_pose.orientation.w = target_quat.w();

    return target_pose;
}

void serviceCall(const geometry_msgs::Pose &odom_msg, const geometry_msgs::Pose &init_msg)

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

    finishreq.trajectory_id = cnt; // int
    if (finishclient.call(finishreq, finishres))
        ROS_INFO("finish_trajectory service success..");

    startreq.configuration_basename = "rplidar.lua";                                                                                 // string
    startreq.configuration_directory = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/lua"; // string
    startreq.use_initial_pose = true;                                                                                                // bool
    // startreq.initial_pose = target_pose; // geometry _msgs
    startreq.initial_pose = Quat2RPY2Quat(odom_msg, init_msg); // geometry _msgs

    startreq.relative_to_trajectory_id = cnt; /// int
    // Call the start_trajectory service
    if (Startclient.call(startreq, startres))
        ROS_INFO("start_trajectory service success..");
    cnt++;
}