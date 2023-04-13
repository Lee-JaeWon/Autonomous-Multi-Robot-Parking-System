/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <QApplication> // Include QApplication header
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include "../include/ui_main/qnode.hpp"
#include <geometry_msgs/PoseStamped.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_main
{

  /*****************************************************************************
  ** Implementation
  *****************************************************************************/

  QNode::QNode(int argc, char **argv) : init_argc(argc),
                                        init_argv(argv)
  {
  }

  QNode::~QNode()
  {
    if (ros::isStarted())
    {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init()
  {
    ros::init(init_argc, init_argv, "ui_main");
    if (!ros::master::check())
    {
      ROS_ERROR("ROS master disconnected");
      return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.

    // ros::NodeHandle n;
    // // Add your ros communications here.
    // chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    start();
    return true;
  }

  void QNode::run()
  {
    ros::Rate loop_rate(10);

    ros::NodeHandle n("~");

    int num_of_robots = 0;
    if (n.getParam("num_of_robots", num_of_robots))
    {
      ROS_INFO("UI_emer node got param: %d", num_of_robots);
    }

    while (ros::ok())
    {
      ros::spinOnce();   // callback 호출
      loop_rate.sleep(); // 루프 주기

      ros::NodeHandle nh;

      if (btn_marker_pub_flag)
      {
        vis_pub = nh.advertise<visualization_msgs::Marker>("Park_marker", 1);
        vis_way = nh.advertise<visualization_msgs::Marker>("way_marker", 1);
        vis_pub_text = nh.advertise<visualization_msgs::Marker>("text_marker", 1);

        ////////////////////////////////////////////////////////
        visualization_msgs::Marker marker;

        uint32_t shape = visualization_msgs::Marker::SPHERE;

        marker.header.frame_id = "robot_1/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 1.8;
        marker.pose.position.y = 1.6;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.69862;
        marker.pose.orientation.w = 0.71548;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f; // yellow
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();

        vis_pub.publish(marker);
        ////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////
        visualization_msgs::Marker marker_waypoint;

        marker_waypoint.header.frame_id = "robot_1/map";
        marker_waypoint.header.stamp = ros::Time::now();
        marker_waypoint.ns = "basic_shapes";
        marker_waypoint.id = 0;
        marker_waypoint.type = shape;
        marker_waypoint.action = visualization_msgs::Marker::ADD;
        marker_waypoint.pose.position.x = 1.8;
        marker_waypoint.pose.position.y = 1.10;
        marker_waypoint.pose.position.z = 0.0;
        marker_waypoint.pose.orientation.x = 0.0;
        marker_waypoint.pose.orientation.y = 0.0;
        marker_waypoint.pose.orientation.z = 0.69862;
        marker_waypoint.pose.orientation.w = 0.71548;
        marker_waypoint.scale.x = 0.2;
        marker_waypoint.scale.y = 0.2;
        marker_waypoint.scale.z = 0.2;
        marker_waypoint.color.r = 0.0f; // blue
        marker_waypoint.color.g = 0.0f;
        marker_waypoint.color.b = 1.0f;
        marker_waypoint.color.a = 1.0;
        marker_waypoint.lifetime = ros::Duration();

        vis_way.publish(marker_waypoint);
        ////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////
        visualization_msgs::Marker marker_text;

        marker_text.header.frame_id = "robot_1/map";
        marker_text.header.stamp = ros::Time::now();
        marker_text.ns = "text_marker";
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;
        marker_text.pose.position.x = 1.8;
        marker_text.pose.position.y = 1.8;
        marker_text.pose.position.z = 0.0;
        marker_text.pose.orientation.x = 0.0;
        marker_text.pose.orientation.y = 0.0;
        marker_text.pose.orientation.z = 0.69862;
        marker_text.pose.orientation.w = 0.71548;
        marker_text.scale.x = 0.2;
        marker_text.scale.y = 0.2;
        marker_text.scale.z = 0.2;

        marker_text.color.r = 0.0f; // Black
        marker_text.color.g = 0.0f;
        marker_text.color.b = 0.0f;
        marker_text.color.a = 1.0;

        marker_text.text = "Parking Lot #1"; // 텍스트 내용

        vis_pub_text.publish(marker_text);

        ////////////////////////////////////////////////////////
      }

      if (btn_goal_flag)
      {
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 1);

        geometry_msgs::PoseStamped goal_msg;

        goal_msg.header.frame_id = "robot_1/map"; // 헤더의 프레임 ID 설정
        goal_msg.header.stamp = ros::Time::now(); // 헤더의 타임 스탬프 설정
        goal_msg.pose.position.x = 1.8;           // 목표 위치의 x 좌표
        goal_msg.pose.position.y = 1.1;           // 목표 위치의 y 좌표

        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = 0.69862;
        goal_msg.pose.orientation.w = 0.71548;

        // 메시지 발행
        goal_pub.publish(goal_msg);

        btn_goal_flag = false;
      }

      if(btn_return_flag){
        return_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 1);

        geometry_msgs::PoseStamped goal_msg_return;

        goal_msg_return.header.frame_id = "robot_1/map"; // 헤더의 프레임 ID 설정
        goal_msg_return.header.stamp = ros::Time::now(); // 헤더의 타임 스탬프 설정
        goal_msg_return.pose.position.x = 0.0;           // 목표 위치의 x 좌표
        goal_msg_return.pose.position.y = 0.0;           // 목표 위치의 y 좌표

        goal_msg_return.pose.orientation.x = 0.0;
        goal_msg_return.pose.orientation.y = 0.0;
        goal_msg_return.pose.orientation.z = 0.0;
        goal_msg_return.pose.orientation.w = 1.0;

        // 메시지 발행
        goal_pub.publish(goal_msg_return);

        btn_return_flag = false;
      }

      if (!ros::master::check())
      {
        ROS_ERROR("ROS master disconnected");
        break;
      }
    }

    // Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

  void QNode::btnExit()
  {
    ROS_WARN("Main UI Shut");
    ROS_WARN("Shutdown process in progress");

    ros::shutdown();
    QApplication::quit();
  }

  void QNode::btnMarkerPub()
  {
    ROS_INFO("btnMarkerPub");
    btn_marker_pub_flag = true;
  }

  void QNode::btnGoalPub()
  {
    ROS_INFO("btnGoalPub");
    btn_goal_flag = true;
  }

  void QNode::btnReturnPub()
  {
    ROS_INFO("btnReturnPub");
    btn_return_flag = true;
  }

} // namespace ui_main
