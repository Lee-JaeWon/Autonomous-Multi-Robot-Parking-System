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
#include <tf/tf.h>
#include <cmath>

/*****************************************************************************
** Namespaces
*****************************************************************************/

double robot_1_x, robot_1_y, robot_1_z;
double robot_1_ox, robot_1_oy, robot_1_oz, robot_1_ow;

void trackCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  robot_1_x = msg->pose.position.x;
  robot_1_y = msg->pose.position.y;
  robot_1_z = msg->pose.position.z;

  robot_1_ox = msg->pose.orientation.x;
  robot_1_oy = msg->pose.orientation.y;
  robot_1_oz = msg->pose.orientation.z;
  robot_1_ow = msg->pose.orientation.w;
}

visualization_msgs::Marker vis_arrow_rviz(double x, double y, double oz, double ow, std::string ns, std::string color)
{
  visualization_msgs::Marker marker;

  uint32_t shape_func = visualization_msgs::Marker::ARROW;

  marker.header.frame_id = ns;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = shape_func;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = oz;
  marker.pose.orientation.w = ow;
  marker.scale.x = 0.4;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  if (color == "yellow")
  {
    marker.color.r = 1.0f; // yellow
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  }
  else if (color == "blue")
  {
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
  }
  else
  {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  }

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  return marker;
}

visualization_msgs::Marker vis_text_rviz(double x, double y, double oz, double ow, std::string ns, std::string text)
{
  visualization_msgs::Marker marker_text;

  marker_text.header.frame_id = ns;
  marker_text.header.stamp = ros::Time::now();
  marker_text.ns = "text_marker";
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;
  marker_text.pose.position.x = x;
  marker_text.pose.position.y = y;
  marker_text.pose.position.z = 0.0;
  marker_text.pose.orientation.x = 0.0;
  marker_text.pose.orientation.y = 0.0;
  marker_text.pose.orientation.z = oz;
  marker_text.pose.orientation.w = ow;
  marker_text.scale.x = 0.2;
  marker_text.scale.y = 0.2;
  marker_text.scale.z = 0.2;

  marker_text.color.r = 0.0f; // Black
  marker_text.color.g = 0.0f;
  marker_text.color.b = 0.0f;
  marker_text.color.a = 1.0;

  marker_text.text = text; // 텍스트 내용

  return marker_text;
}

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
    ros::Rate loop_rate(100);

    ros::NodeHandle nh;

    tracked_sub = nh.subscribe(track_ns_robot1, 100, trackCallBack);
    chatter_emer = nh.advertise<std_msgs::Bool>("/emer_flag", 1000);

    while (ros::ok())
    {
      ros::spinOnce();   // callback 호출
      loop_rate.sleep(); // 루프 주기

      if (btn_marker_pub_flag) // goal marker
      {
        vis_pub_one = nh.advertise<visualization_msgs::Marker>("Park_one_marker", 1);
        vis_one_way = nh.advertise<visualization_msgs::Marker>("way_one_marker", 1);
        vis_pub_one_text = nh.advertise<visualization_msgs::Marker>("text_one_marker", 1);

        vis_pub_two = nh.advertise<visualization_msgs::Marker>("Park_two_marker", 1);
        vis_two_way = nh.advertise<visualization_msgs::Marker>("way_two_marker", 1);
        vis_pub_two_text = nh.advertise<visualization_msgs::Marker>("text_two_marker", 1);

        ////////////////////////////////////////////////////////

        visualization_msgs::Marker marker_temp;

        marker_temp = vis_arrow_rviz(one_goal_x, one_goal_y, one_goal_oz, one_goal_ow, robot_1_map, "yellow");
        vis_pub_one.publish(marker_temp);

        marker_temp = vis_arrow_rviz(one_pre_goal_x, one_pre_goal_y, one_pre_goal_oz, one_pre_goal_ow, robot_1_map, "blue");
        vis_one_way.publish(marker_temp);

        marker_temp = vis_arrow_rviz(two_goal_x, two_goal_y, two_goal_oz, two_goal_ow, robot_1_map, "yellow");
        vis_pub_two.publish(marker_temp);

        marker_temp = vis_arrow_rviz(two_pre_goal_x, two_pre_goal_y, two_pre_goal_oz, two_pre_goal_ow, robot_1_map, "blue");
        vis_two_way.publish(marker_temp);

        marker_temp = vis_text_rviz(text_first_goal_x, text_first_goal_y, text_first_goal_oz, text_first_goal_ow, robot_1_map, "Parking Lot #1");
        vis_pub_one_text.publish(marker_temp);

        marker_temp = vis_text_rviz(text_two_goal_x, text_two_goal_y, text_two_goal_oz, text_two_goal_ow, robot_1_map, "Parking Lot #2");
        vis_pub_two_text.publish(marker_temp);
      
        btn_marker_pub_flag = false;
      }

      if (btn_goal_flag)
      {
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 1);

        geometry_msgs::PoseStamped goal_msg;

        goal_msg.header.frame_id = "map";  // 헤더의 프레임 ID 설정
        goal_msg.header.stamp = ros::Time::now();  // 헤더의 타임 스탬프 설정
        goal_msg.pose.position.x = one_pre_goal_x; // 목표 위치의 x 좌표
        goal_msg.pose.position.y = one_pre_goal_y; // 목표 위치의 y 좌표

        goal_msg.pose.orientation.x = 0.0;
        goal_msg.pose.orientation.y = 0.0;
        goal_msg.pose.orientation.z = one_pre_goal_oz;
        goal_msg.pose.orientation.w = one_pre_goal_ow;

        // 메시지 발행
        goal_pub.publish(goal_msg);

        btn_goal_flag = false;
      }

      if (btn_return_flag)
      {
        return_pub = nh.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 1);

        geometry_msgs::PoseStamped goal_msg_return;

        goal_msg_return.header.frame_id = "map"; // 헤더의 프레임 ID 설정
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

      if (btn_poseset_flag) // 버튼 눌림시
      {
        tf::Quaternion Quat( // 각 변환
            robot_1_ox,
            robot_1_oy,
            robot_1_oz,
            robot_1_ow);
        tf::Matrix3x3 m(Quat);
        m.getRPY(roll, pitch, yaw);

        tf::Quaternion Quat2( // 각 변환
            one_goal_ox,
            one_goal_oy,
            one_goal_oz,
            one_goal_ow);
        tf::Matrix3x3 m2(Quat2);
        m2.getRPY(roll2, pitch2, yaw2);

        cmd_stop_pub_1 = nh.advertise<std_msgs::Bool>("/robot_1/cmd_vel_stop_tomux", 1000);
        std_msgs::Bool msg_temp;

        // 본 명령 수행
        pub_vel = nh.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 1);
        geometry_msgs::Twist cmd_vel;
        if (stop_flag)
        {
          ROS_INFO_ONCE("cmd_stop_pub_1 flag to mux");

          msg_temp.data = true;
          cmd_stop_pub_1.publish(msg_temp);
          cmd_vel.linear.x = 0.0;
          cmd_vel.angular.z = 0.0;

          stop_flag = false;
        }

        if (rotate_flag)
        {
          double sign = yaw - yaw2;
          double range = abs(yaw - yaw2);
          if (range_flag)
          {
            ROS_INFO_ONCE("Now in range_flag");
            // ROS_INFO("Now in range_flag");

            if (range < 0.05)
            {
              cmd_vel.linear.x = 0.0;
              cmd_vel.angular.z = 0.0;
              ROS_INFO("Pose set finish");
              dist_flag = true;   // 진입 1
              range_flag = false; // 탈출 1
            }
            else
            {
              if (sign < 0)
              {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.4;
              }
              else
              {
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.4;
              }
            }
          } // 탈출 1

          // double dist_range = sqrt((goal_x - robot_1_x) * (goal_x - robot_1_x) + (goal_y - robot_1_y) * (goal_y - robot_1_y));
          double dist_range = abs(one_goal_y - robot_1_y); // 수정요구
          if (dist_flag)                                   // 진입 1
          {
            // ROS_INFO("Now in dist_flag");
            ROS_INFO_ONCE("Now in dist_flag");
            cmd_vel.linear.x = 0.1;
            cmd_vel.angular.z = 0.0;

            if (dist_range < 0.04)
            {
              ROS_INFO("distance set finish");

              cmd_vel.linear.x = 0.0;
              cmd_vel.angular.z = 0.0;

              // traker 복구
              msg_temp.data = false;
              cmd_stop_pub_1.publish(msg_temp);

              dist_flag = false;
              rotate_flag = false;
              btn_return_flag = false;
            }
            else
            {
              msg_temp.data = true;
              cmd_stop_pub_1.publish(msg_temp);
            }
          }
          pub_vel.publish(cmd_vel);
          // ROS_INFO("range_flag %d, range %f", range_flag, range);
        }
      }

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

  void QNode::Q_btnPoseset()
  {
    ROS_INFO("Q_btnPoseset");
    stop_flag = true;
    rotate_flag = true;
    range_flag = true;
    dist_flag = false;
    btn_poseset_flag = true;
  }

  void QNode::Q_btnemerset()
  {
    ROS_INFO("Q_btnemerset");
    emer_btn_flag = true;
  }

} // namespace ui_main
