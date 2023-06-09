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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "../include/ui_emergency/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_emergency
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
    ros::init(init_argc, init_argv, "ui_emergency");
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
    n.getParam("num_of_robots", num_of_robots);
    n.getParam("robot_1_init_x",robot_1_x);
    n.getParam("robot_1_init_y",robot_1_y);
    n.getParam("robot_1_init_a",robot_1_a);
    n.getParam("robot_2_init_x",robot_2_x);
    n.getParam("robot_2_init_y",robot_2_y);
    n.getParam("robot_2_init_a",robot_2_a);
    n.getParam("robot_3_init_x",robot_3_x);
    n.getParam("robot_3_init_y",robot_3_y);
    n.getParam("robot_3_init_a",robot_3_a);

    ros::NodeHandle nh;
    initpose_one = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_1/initialpose", 1000);
    initpose_two = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_2/initialpose", 1000);
    initpose_thr = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/robot_3/initialpose", 1000);

    geometry_msgs::PoseWithCovarianceStamped msg_one;
    geometry_msgs::PoseWithCovarianceStamped msg_two;
    geometry_msgs::PoseWithCovarianceStamped msg_thr;
    msg_one.pose.pose.position.x = robot_1_x;
    msg_one.pose.pose.position.y = robot_1_y;
    msg_one.pose.pose.orientation.w = 1.0;

    msg_two.pose.pose.position.x = robot_2_x;
    msg_two.pose.pose.position.y = robot_2_y;
    msg_two.pose.pose.orientation.w = 1.0;

    msg_thr.pose.pose.position.x = robot_3_x;
    msg_thr.pose.pose.position.y = robot_3_y;
    msg_thr.pose.pose.orientation.w = 1.0;

    chatter_emer = nh.advertise<std_msgs::Bool>("/emer_flag", 1000);

    while (ros::ok())
    {
      ros::spinOnce();   // callback 호출
      loop_rate.sleep(); // 루프 주기

      if (emer_btn_flag)
      {
        std_msgs::Bool msg1;
        msg1.data = false;
        chatter_emer.publish(msg1);
      }
      else
      {
        std_msgs::Bool msg1;
        msg1.data = true;
        chatter_emer.publish(msg1);
      }

      if (poseinit_flag)
      {
        initpose_one.publish(msg_one);
        initpose_two.publish(msg_two);
        initpose_thr.publish(msg_thr);
        poseinit_flag = false;
      }

      if (!ros::master::check())
      {
        ROS_ERROR("ROS master disconnected");
        break;
      }
    }

    // Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

  void QNode::btnClicked()
  {
    ROS_WARN("System emergency has been requested.");
    emer_btn_flag = true;
  }

  void QNode::btnExit()
  {
    ROS_WARN("Emergency UI Shut");

    ros::shutdown();
    QApplication::quit();
  }

  void QNode::btnInitPose()
  {
    ROS_INFO("Robot Pose Initialize");
    poseinit_flag = true;
  }

} // namespace ui_emergency
