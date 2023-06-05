#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
<<<<<<< HEAD
#include <tf2_ros/transform_listener.h>
=======
#include <tf/transform_listener.h>
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
// #include <opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"

using namespace cv;
using namespace std;

//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
ros::Subscriber odom_sub;
// Publisher
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher local_goal_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;
geometry_msgs::PoseStamped path_point;



// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;
bool traked_flag = false;
int rate;
int origin_x, origin_y;
double resol;
int mx, my;
double present_x, present_y;
<<<<<<< HEAD
=======
double robot_x ;
double robot_y ;
std::string s;
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    // Get parameter
    resol = msg.info.resolution;
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height - i - 1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);
    // Set flag
    map_flag = true;
    ROS_INFO("Map Received");

}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{

    //    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
    //             startPoint.x, startPoint.y);
}

void TargetPointtCallback(const geometry_msgs::PoseStamped &msg)
{
<<<<<<< HEAD
=======
    traked_flag = true;
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);
    // Set flag
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

    //    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
    //             targetPoint.x, targetPoint.y);
}
<<<<<<< HEAD

void odomCallback(const nav_msgs::Odometry &msg)
{
    
    ROS_INFO("Odom Received!");
    // Point2d src_point = Point2d(robot.x, robot.y);
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);
    // Set flag
    startpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag) start_flag = true;

=======
void odomCallback(const nav_msgs::Odometry &msg)
{

    // try{
    //   listener.lookupTransform("/map","/robot_1/base_link",  
    //                            ros::Time(0), transform);
    // }
    // catch (tf::TransformException ex){
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    if (traked_flag)
    {
        // mx = int((msg.pose.pose.position.x - origin_x) / (resol)); // world to map
        // my = int((msg.pose.pose.position.y - origin_y) / (resol)); // world to map

        Point2d src_point = Point2d(robot_x, robot_y);
        // Point2d src_point = Point2d(transform.getOrigin().x(), transform.getOrigin().y());

        OccGridParam.Map2ImageTransform(src_point, startPoint);

        // Set flag
        startpoint_flag = true;
        if (map_flag && startpoint_flag && targetpoint_flag)
        {
            // start_flag = true;
        }
    }
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
}

//-------------------------------- Main function ---------------------------------//
int main(int argc, char *argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
<<<<<<< HEAD

    tf::TransformListener listener;
    tf::StampedTransform transform;

=======
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
    ROS_INFO("Start astar node!\n");

    // param check

    if (ros::param::get("~namespace", s))
        ROS_INFO("Astar node got param: %s", s.c_str());
    else
        ROS_ERROR("Astar Failed to get param '%s'", s.c_str());

    // Initial variables
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);

<<<<<<< HEAD
    // namespace
    std::string path_map_ns = "/path_map"; // fixed frame for path_planning
    std::string map_ns = "/map"; // fixed frame

    std::string goal_ns = s + "/move_base_simple/goal";
    std::string odom_ns = s + "/odom";
    std::string initpose_ns = s + "/initialpose";

    // Subscribe topics
    map_sub = nh.subscribe(path_map_ns, 10, MapCallback);
=======
    // Subscribe topics
    std::string map_ns = "map"; // fixed frame
    std::string goal_ns = s + "/move_base_simple/goal";
    std::string track_ns = s + "/odom";
    std::string initpose_ns = s + "/initialpose";

    map_sub = nh.subscribe(s + "/map", 10, MapCallback);
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
    // startPoint_sub = nh.subscribe(initpose_ns, 10, StartPointCallback);
    targetPoint_sub = nh.subscribe(goal_ns, 10, TargetPointtCallback);
    odom_sub = nh.subscribe(odom_ns, 10, odomCallback);

    // Advertise topics
<<<<<<< HEAD
    std::string path_ns = s + "/global_path";
=======
    std::string path_ns = s + "/path";
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1); // potential problem
>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
    path_pub = nh.advertise<nav_msgs::Path>(path_ns, 10);
    // Loop and wait for callback

    ros::Rate loop_rate(rate);
    while (ros::ok())

    {
<<<<<<< HEAD
        if (start_flag) {
=======

        try{
        listener.lookupTransform("map", s +"/base_link",  
                                ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());

        ros::Duration(1.0).sleep();
        }
        robot_x = transform.getOrigin().x();
        robot_y = transform.getOrigin().y();

        if (start_flag)
        {    
            ROS_INFO("START");

>>>>>>> af0e7d86847e6bec22eb3f1cee44483a15ec3165
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;
            try{
                listener.lookupTransform("/map", s + "/base_link",ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            geometry_msgs::Point robot;
            robot.x = transform.getOrigin().x();
            robot.y = transform.getOrigin().y();

            Point2d src_point = Point2d(robot.x, robot.y);
            OccGridParam.Map2ImageTransform(src_point, startPoint);

            astar.PathPlanning(startPoint, targetPoint, PathList);
            if (!PathList.empty()) {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = map_ns;
                path.poses.clear();
                for (int i = 0; i < PathList.size(); i++) {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = map_ns;
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();

                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }
            int i = 0;
            if ((path_point.pose.position.x - present_x) + (path_point.pose.position.y - present_y) > 0.3)
                i++;
            ROS_INFO("%d\n", i);
            path_point = path.poses[20];
            path_point.pose.orientation.w = 1;
            path_point.pose.orientation.x = 0;
            path_point.pose.orientation.y = 0;
            path_point.pose.orientation.z = 0;

            //  local_goal_pub.publish(path_point);

            // Set flag
            start_flag = false;
            targetpoint_flag = false;
        }

        if (map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}