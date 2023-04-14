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
    if (n.getParam("num_of_robots", num_of_robots))
    {
      ROS_INFO("UI_emer node got param: %d", num_of_robots);
    }

    while (ros::ok())
    {
      ros::spinOnce();   // callback 호출
      loop_rate.sleep(); // 루프 주기

      ros::NodeHandle nh;
      chatter_emer = nh.advertise<std_msgs::Bool>("/emer_flag", 1000);

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

} // namespace ui_emergency
