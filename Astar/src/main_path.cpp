#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include <time.h>


using namespace cv;
using namespace std;

//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;

ros::Subscriber targetPoint1_sub;
ros::Subscriber targetPoint2_sub;
ros::Subscriber targetPoint3_sub;

vector<Point> R1PathList;
vector<Point> R2PathList;
vector<Point> R3PathList;

vector<double> R1PathTime;
vector<double> R2PathTime;
vector<double> R3PathTime;


ros::Subscriber odom_sub1;
ros::Subscriber odom_sub2;
ros::Subscriber odom_sub3;

// Publisher
ros::Publisher mask_pub;

ros::Publisher path_pub1;
ros::Publisher path_pub2;
ros::Publisher path_pub3;

ros::Publisher point_pub1;
ros::Publisher point_pub2;


ros::Publisher local_goal_pub;

// Object
nav_msgs::OccupancyGrid OccGridMask;
geometry_msgs::PoseStamped path_point;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint1, targetPoint1;
Point startPoint2, targetPoint2;
Point startPoint3, targetPoint3;


// Parameter
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;

double Time = 0.0;

bool R1 = false;
bool R2 = false;
bool R3 = false;

//bool traked_flag = false;
bool start_flag;
int rate;
int origin_x, origin_y;
double resol;
double present_x,present_y;
int mx, my;

double min_(double a , double b){
    if (a > b) return b ;
    else return a;
}

double calc_path_time(vector<Point> pt1 ,vector<Point> pt2,vector<Point> pt3, int curp){

    geometry_msgs::PointStamped t1;
    geometry_msgs::PointStamped t2;

    Point2d r1pointt;
    Point2d r2pointt;
    Point2d r3pointt;


    double minT = 100000.0 ;
    double minDist = 100000.0;
    double time_diff1 = 100000.0;
    double time_diff2 = 100000.0;
    int R1p , R2p, R3p = 0;

    ROS_INFO("Calc_path_time");
    if (curp == 1){
        ROS_INFO("first path publish");
        if (!pt2.empty()){
            for (int i = 0 ; i < pt1.size() ; i ++){
                for (int j = 0; j < pt2.size() ; j ++){

                    OccGridParam.Image2MapTransform(R1PathList[i], r1pointt);
                    OccGridParam.Image2MapTransform(R2PathList[j], r2pointt);
                    double dst  = sqrt(pow(r1pointt.x - r2pointt.x,2)+pow(r1pointt.y - r2pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R1p = i;
                        R2p = j;
                    }
                }
            }
            time_diff1  = R1PathTime[R1p] - R2PathTime[R2p];
            if(abs(time_diff1) > 10.0) minDist = 100000.0;
        }
        if (!pt3.empty()){
            for (int i = 0 ; i < pt1.size() ; i ++){
                for (int j = 0; j < pt3.size() ; j ++){

                    OccGridParam.Image2MapTransform(R1PathList[i], r1pointt);
                    OccGridParam.Image2MapTransform(R3PathList[j], r3pointt);
                    double dst  = sqrt(pow(r1pointt.x - r3pointt.x,2)+pow(r1pointt.y - r3pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R1p = i;
                        R3p = j;
                    }
                }
            }
            //
            time_diff2  = R1PathTime[R1p] - R3PathTime[R3p];
        }
        // ROS_INFO("%f %f",time_diff1 , time_diff2);
        double time_diff = min_(time_diff1 , time_diff2);
        ROS_INFO("time_diff : %lf minDist : %f",time_diff, minDist);
        if ( (( time_diff  > 0.0 )&&(time_diff) < 5.0) && minDist < 0.3) return abs(time_diff);
        else return 0.0;
    }

    else if (curp == 2){
        ROS_INFO("second path publish");
        if (!pt1.empty()){
            for (int i = 0 ; i < pt2.size() ; i ++){
                for (int j = 0; j < pt1.size() ; j ++){
                    OccGridParam.Image2MapTransform(R2PathList[i], r2pointt);
                    OccGridParam.Image2MapTransform(R1PathList[j], r1pointt);
                    double dst  = sqrt(pow(r1pointt.x - r2pointt.x,2)+pow(r1pointt.y - r2pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R2p = i;
                        R1p = j;
                        }
                    }
                }
        //
            time_diff1  = R2PathTime[R2p] - R1PathTime[R1p];
            if(abs(time_diff1) > 10.0) minDist = 100000.0;
        }
        if (!pt3.empty()){
            for (int i = 0 ; i < pt2.size() ; i ++){
                for (int j = 0; j < pt3.size() ; j ++){
                    OccGridParam.Image2MapTransform(R2PathList[i], r2pointt);
                    OccGridParam.Image2MapTransform(R3PathList[j], r3pointt);
                    double dst  = sqrt(pow(r2pointt.x - r3pointt.x,2)+pow(r2pointt.y - r3pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R2p = i;
                        R3p = j;
                        }
                    }
                }
            //
            time_diff2  = R2PathTime[R2p] - R3PathTime[R3p];
        }
        double time_diff = min_(time_diff1 , time_diff2);
        ROS_INFO("time_diff : %lf minDist : %f",time_diff, minDist);

        if ((( time_diff  > 0.0 )&&(time_diff) < 5.0) && minDist < 0.3) return abs(time_diff);
        else return 0.0;
    }
    else if (curp == 3){
        ROS_INFO("third path publish");
        if (!pt1.empty()){
            for (int i = 0 ; i < pt3.size() ; i ++){
                for (int j = 0; j < pt1.size() ; j ++){
                    OccGridParam.Image2MapTransform(R3PathList[i], r3pointt);
                    OccGridParam.Image2MapTransform(R1PathList[j], r1pointt);
                    double dst  = sqrt(pow(r3pointt.x - r1pointt.x,2) + pow(r3pointt.y - r1pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R3p = i;
                        R1p = j;
                    }
                }
            }
        //
            time_diff1  = R3PathTime[R3p] - R1PathTime[R1p];
            if(abs(time_diff1) > 10.0) minDist = 100000.0;
        }
        if (!pt2.empty()){
                
            for (int i = 0 ; i < pt3.size() ; i ++){
                for (int j = 0; j < pt2.size() ; j ++){
                    OccGridParam.Image2MapTransform(R3PathList[i], r3pointt);
                    OccGridParam.Image2MapTransform(R2PathList[j], r2pointt);
                    double dst  = sqrt(pow(r2pointt.x - r3pointt.x,2)+pow(r2pointt.y - r3pointt.y,2));
                    if(dst < minDist){
                        minDist = dst;
                        R3p = i;
                        R2p = j;
                        }
                    }
                }
            //
            time_diff2  = R3PathTime[R3p] - R2PathTime[R2p];
        }
        double time_diff = min_(time_diff1 , time_diff2);
        ROS_INFO("time_diff : %lf minDist : %f",time_diff, minDist);
        if ((( time_diff  > 0.0 )&&(time_diff) < 5.0) && minDist < 0.3) return abs(time_diff);
        else return 0.0;
    }




    // OccGridParam.Image2MapTransform(R1PathList[R1p], r1pointt);
    // OccGridParam.Image2MapTransform(R2PathList[R2p], r2pointt);

    // t1.header.frame_id="map";
    // t1.header.stamp = ros::Time::now();
    // t1.point.x = r1pointt.x;
    // t1.point.y = r1pointt.y;

    // t2.header.frame_id="map";
    // t2.header.stamp = ros::Time::now();
    // t2.point.x = r2pointt.x;
    // t2.point.y = r2pointt.y;

    // point_pub1.publish(t1);
    // point_pub2.publish(t2);

    // ROS_INFO("min dist: %f",minDist);
    // ROS_INFO("time diff: %lf", (R1PathTime[R1p] - R2PathTime[R2p]) );
    // ROS_INFO("robot1 time is : %lf ",(R1PathTime[R1p] - R1PathTime[0]));
    // ROS_INFO("robot2 time is : %lf ",(R2PathTime[R2p] - R2PathTime[0]));
    // return minT;


    }
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

    // Publish Mask
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            OccProb = Mask.at<uchar>(height - i - 1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{

    //    ROS_INFO("startPoint: %f %f %d %d", msg.pose.pose.position.x, msg.pose.pose.position.y,
    //             startPoint.x, startPoint.y);
}

void Target1PointtCallback(const geometry_msgs::PoseStamped &msg)
{
    //traked_flag = true;
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint1);
    R1 = true;
    // Set flag
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

    //    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
    //             targetPoint.x, targetPoint.y);
}


void Target2PointtCallback(const geometry_msgs::PoseStamped &msg)
{
    //traked_flag = true;
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint2);
    R2 = true;
    // Set flag
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

    //    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
    //             targetPoint.x, targetPoint.y);
}

void Target3PointtCallback(const geometry_msgs::PoseStamped &msg)
{
    //traked_flag = true;
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint3);
    R3 = true;
    // Set flag
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }

    //    ROS_INFO("targetPoint: %f %f %d %d", msg.pose.position.x, msg.pose.position.y,
    //             targetPoint.x, targetPoint.y);
}


void odom1Callback(const nav_msgs::Odometry &msg)
{
   // if (traked_flag)
   // {
        Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
        OccGridParam.Map2ImageTransform(src_point, startPoint1);
        // Set flag
        startpoint_flag = true;
        if (map_flag && startpoint_flag && targetpoint_flag)
        {
            start_flag = true;
        }
   // }
   // traked_flag = false;
}

void odom2Callback(const nav_msgs::Odometry &msg)
{
   // if (traked_flag)
   // {
        Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
        OccGridParam.Map2ImageTransform(src_point, startPoint2);
        // Set flag
        startpoint_flag = true;
        if (map_flag && startpoint_flag && targetpoint_flag)
        {
            start_flag = true;
        }
   // }
   // traked_flag = false;
}

void odom3Callback(const nav_msgs::Odometry &msg)
{
   // if (traked_flag)
   // {
        Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
        OccGridParam.Map2ImageTransform(src_point, startPoint3);


        // Set flag
        startpoint_flag = true;
        if (map_flag && startpoint_flag && targetpoint_flag)
        {
            start_flag = true;
        }
   // }
   // traked_flag = false;
}



//-------------------------------- Main function ---------------------------------//
int main(int argc, char *argv[])
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

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

    // Subscribe topics
    map_sub = nh.subscribe("/path_map", 10, MapCallback);
    startPoint_sub = nh.subscribe("initialpose", 10, StartPointCallback);

    targetPoint1_sub = nh.subscribe("/robot1/move_base_simple/goal", 10, Target1PointtCallback);
    targetPoint2_sub = nh.subscribe("/robot2/move_base_simple/goal", 10, Target2PointtCallback);
    targetPoint3_sub = nh.subscribe("/robot3/move_base_simple/goal", 10, Target3PointtCallback);

    odom_sub1 = nh.subscribe("/robot1/odom", 10, odom1Callback);
    odom_sub2 = nh.subscribe("/robot2/odom", 10, odom2Callback);
    odom_sub3 = nh.subscribe("/robot3/odom", 10, odom3Callback);


    // Advertise topics
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    path_pub1 = nh.advertise<nav_msgs::Path>("nav_path1", 10);
    path_pub2 = nh.advertise<nav_msgs::Path>("nav_path2", 10);
    path_pub3 = nh.advertise<nav_msgs::Path>("nav_path3", 10);

    point_pub1 = nh.advertise<geometry_msgs::PointStamped>("p1", 10);
    point_pub2 = nh.advertise<geometry_msgs::PointStamped>("p2", 10);

    // Loop and wait for callback
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {   
        Time += 0.1;
        if (start_flag)
        {
            double start_time = ros::Time::now().toSec();
            // Start planning path
            vector<Point> PathList;

            if (R1){
                astar.PathPlanning(startPoint1, targetPoint1, PathList);
                R1PathList.clear(); R1PathTime.clear();
                R1PathList = PathList;
                }
            if (R2){
                astar.PathPlanning(startPoint2, targetPoint2, PathList);
                R2PathTime.clear(); R2PathList.clear();
                R2PathList = PathList;
                }
            if (R3){
                astar.PathPlanning(startPoint3, targetPoint3, PathList);
                R3PathList = PathList; R3PathTime.clear();
                R3PathTime.clear();
                }

            if (!PathList.empty()){

                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();

                for (int i = 0; i < PathList.size(); i++){

                    Point2d dst_point;
                    geometry_msgs::PoseStamped pose_stamped;

                    OccGridParam.Image2MapTransform(PathList[i], dst_point);

                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);

                    if (R1) R1PathTime.push_back(Time + 0.03333 * i);
                    if (R2) R2PathTime.push_back(Time + 0.03333 * i);
                    if (R3) R3PathTime.push_back(Time + 0.03333 * i);
                }

                double minTime = 0.0;
                
                if (R1){
                    minTime = calc_path_time(R1PathList, R2PathList, R3PathList,1);
                    for(int t = 0 ; t < R1PathTime.size() ; t++) R1PathTime[t] += minTime;
                    ros::Duration(minTime).sleep();
                    path_pub1.publish(path);
                    }
                else if(R2){
                    minTime = calc_path_time(R1PathList, R2PathList, R3PathList,2);
                    for(int t = 0 ; t < R2PathTime.size() ; t++) R2PathTime[t] += minTime;
                    ros::Duration(minTime).sleep();
                    path_pub2.publish(path);
                    }
                else if(R3){
                    minTime = calc_path_time(R1PathList, R2PathList, R3PathList,3);
                    for(int t = 0 ; t < R3PathTime.size() ; t++) R3PathTime[t] += minTime;
                    ros::Duration(minTime).sleep();
                    path_pub3.publish(path);
                    }

                ROS_INFO("Time is : %lf",minTime);
                double end_time = ros::Time::now().toSec();
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }

            // Set flag
            start_flag = false;
            targetpoint_flag = false;
            R1 = false; R2 = false; R3 = false;

        }         

        loop_rate.sleep();
        ros::spinOnce();

    }

    return 0;


}
