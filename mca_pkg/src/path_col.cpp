#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#define ROBOT_NUM 3

int path_one_size = 0;
int path_two_size = 0;
int path_thr_size = 0;

bool onetwo_meet = false;
bool onethr_meet = false;
bool twothr_meet = false;

nav_msgs::Path path_one;
nav_msgs::Path path_two;
nav_msgs::Path path_thr;

double robot_one_x = 0.0;
double robot_one_y = 0.0;
double robot_two_x = 0.0;
double robot_two_y = 0.0;
double robot_thr_x = 0.0;
double robot_thr_y = 0.0;

double goal_one_x = 0.0;
double goal_one_y = 0.0;
double goal_two_x = 0.0;
double goal_two_y = 0.0;
double goal_thr_x = 0.0;
double goal_thr_y = 0.0;

double dist_one = 0.0;
double dist_two = 0.0;
double dist_thr = 0.0;

double dist_onetwo = 0.0;
double dist_onethr = 0.0;
double dist_twothr = 0.0;

double onetwo_conflict_x = 0.0;

void PathOneCallback(const nav_msgs::Path::ConstPtr &path_)
{
    path_one = *path_;
    path_one_size = (int)path_one.poses.size();
}

void PathTwoCallback(const nav_msgs::Path::ConstPtr &path_)
{
    path_two = *path_;
    path_two_size = (int)path_two.poses.size();
}

void PathThrCallback(const nav_msgs::Path::ConstPtr &path_)
{
    path_thr = *path_;
    path_thr_size = (int)path_thr.poses.size();
}

void GoalOneCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_one_x = msg->pose.position.x;
    goal_one_y = msg->pose.position.y;
}

void GoalTwoCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_two_x = msg->pose.position.x;
    goal_two_y = msg->pose.position.y;
}

void GoalThrCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_thr_x = msg->pose.position.x;
    goal_thr_y = msg->pose.position.y;
}

void hasConflict()
{
    for (const auto &pose_one : path_one.poses)
    {
        for (const auto &pose_two : path_two.poses)
        {
            // ROS_INFO("x: %f, y: %f, z: %f", pose_one.pose.position.x, pose_one.pose.position.y, pose_one.pose.position.z);
            if (pose_one.pose == pose_two.pose)
            {
                ROS_INFO("Find same element! one-two");
                onetwo_meet = true;
            }
        }

        for (const auto &pose_thr : path_thr.poses)
        {
            if (pose_one.pose == pose_thr.pose)
            {
                ROS_INFO("Find same element! one-thr");
                onethr_meet = true;
            }
        }
    }

    for (const auto &pose_two : path_two.poses)
    {
        for (const auto &pose_thr : path_thr.poses)
        {
            if (pose_two.pose == pose_thr.pose)
            {
                ROS_INFO("Find same element! two-thr");
                twothr_meet = true;
            }
        }
    }
}

void tf_transform()
{
    tf::TransformListener listener_one;
    tf::TransformListener listener_two;
    tf::TransformListener listener_thr;

    tf::StampedTransform transform_one;
    tf::StampedTransform transform_two;
    tf::StampedTransform transform_thr;

    try
    {
        listener_one.lookupTransform("map", "robot_1/base_link",
                                     ros::Time(0), transform_one);
        robot_one_x = transform_one.getOrigin().x();
        robot_one_y = transform_one.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    try
    {
        listener_two.lookupTransform("map", "robot_2/base_link",
                                     ros::Time(0), transform_two);
        robot_two_x = transform_two.getOrigin().x();
        robot_two_y = transform_two.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    try
    {
        listener_thr.lookupTransform("map", "robot_3/base_link",
                                     ros::Time(0), transform_thr);
        robot_thr_x = transform_thr.getOrigin().x();
        robot_thr_y = transform_thr.getOrigin().y();
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void calc_dist() 
{
    /* Distance between Robot and Goal */
    dist_one = sqrt((goal_one_x - robot_one_x) * (goal_one_x - robot_one_x) + (goal_one_y - robot_one_y) * (goal_one_y - robot_one_y));
    dist_two = sqrt((goal_two_x - robot_two_x) * (goal_two_x - robot_two_x) + (goal_two_y - robot_two_y) * (goal_two_y - robot_two_y));
    dist_thr = sqrt((goal_thr_x - robot_thr_x) * (goal_thr_x - robot_thr_x) + (goal_thr_y - robot_thr_y) * (goal_thr_y - robot_thr_y));

    dist_onetwo = sqrt((robot_one_x - robot_two_x) * (robot_one_x - robot_two_x) + (robot_one_y - robot_two_y) * (robot_one_y - robot_two_y));
    dist_onethr = sqrt((robot_one_x - robot_thr_x) * (robot_one_x - robot_thr_x) + (robot_one_y - robot_thr_y) * (robot_one_y - robot_thr_y));
    dist_twothr = sqrt((robot_two_x - robot_thr_x) * (robot_two_x - robot_thr_x) + (robot_two_y - robot_thr_y) * (robot_two_y - robot_thr_y));
}

void select_act()
{
    if (onetwo_meet)
    {
        ROS_INFO("dist_one : %f, dist_two : %f", dist_one, dist_two);
        if(dist_one < dist_two)
        {
            
        }
    }
    if (twothr_meet)
    {
        ROS_INFO("dist_two : %f, dist_thr : %f", dist_two, dist_thr);
    }
    if (onethr_meet)
    {
        ROS_INFO("dist_one : %f, dist_thr : %f", dist_one, dist_thr);
    }
}

void reset_Bool()
{
    onethr_meet = false;
    onetwo_meet = false;
    twothr_meet = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Collision_Avoidance");

    int hz = 10;

    ros::NodeHandle nh;
    ros::Rate rate(hz);
    ros::NodeHandle n("~");

    ros::Subscriber path_sub_one;
    ros::Subscriber path_sub_two;
    ros::Subscriber path_sub_thr;

    ros::Subscriber goal_sub_one;
    ros::Subscriber goal_sub_two;
    ros::Subscriber goal_sub_thr;

    path_sub_one = nh.subscribe("/robot_1/path", 1, PathOneCallback);
    path_sub_two = nh.subscribe("/robot_2/path", 1, PathTwoCallback);
    path_sub_thr = nh.subscribe("/robot_3/path", 1, PathThrCallback);

    goal_sub_one = nh.subscribe("/robot_1/move_base_simple/goal", 1, GoalOneCallback);
    goal_sub_one = nh.subscribe("/robot_2/move_base_simple/goal", 1, GoalTwoCallback);
    goal_sub_one = nh.subscribe("/robot_3/move_base_simple/goal", 1, GoalThrCallback);

    while (ros::ok())
    {
        hasConflict(); // 충돌 검사

        tf_transform(); // 로봇 위치 변환

        calc_dist(); // 로봇과 goal 위치 계산

        select_act(); // 필요한 action 결정

        reset_Bool(); // flag 초기화

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

// 1. 패스가 겹치면
// 어떻게 할 것인가? -> 로봇 거리 계산해서 일정 거리 이내이면 한 놈 먼저 (이상적인 경우,x자)
// 마주보고 있는 형태여서 거의 모든 path가 겹친다면? ->