#include "pathTracking/tracker.h"

#include "stanley.cpp"

#include "kanayama.cpp"

std::string s;

void pathCallback(const nav_msgs::Path::ConstPtr &path_)
{
  path = *path_;
  trajectiory_subscribed = true;
}

void emer_Callback(const std_msgs::Bool::ConstPtr &msg)
{
  emer_flag = msg->data;
}

void tf_Listener(std::string map_ns, std::string baselink_ns)
{
  geometry_msgs::TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer.lookupTransform(map_ns, baselink_ns,
                                                ros::Time(0));
    pub.publish(transformStamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    tf_listened = false;
    //    ros::Duration(1.0).sleep();
    //    continue;
    return;
  }

  tf_listened = true;

  // Find heading angle of robot
  tf2::Quaternion quaternion(
      transformStamped.transform.rotation.x,
      transformStamped.transform.rotation.y,
      transformStamped.transform.rotation.z,
      transformStamped.transform.rotation.w);

  double roll, pitch, yaw;

  tf2::Matrix3x3 m(quaternion);
  m.getRPY(roll, pitch, yaw);

  // getting from x and y coordinate of robot in global frame
  robot_X = transformStamped.transform.translation.x;
  robot_Y = transformStamped.transform.translation.y;
  robot_Yaw = yaw;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker");

  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  // get Parameters
  n.getParam("hz", hz);
  n.getParam("tracker", tracker_name);

  // Param - Stanley
  n.getParam("k", k);
  n.getParam("k2", k2);
  n.getParam("v", v);

  // Param - Kanayama
  n.getParam("k_x", k_x);
  n.getParam("k_y", k_y);
  n.getParam("timeStep", timeStep);

  // param check

  if (ros::param::get("~namespace", s))
  {
    ROS_INFO("Traker node got param: %s", s.c_str());
    s += '/';
  }
  else
    ROS_ERROR("Traker Failed to get param '%s'", s.c_str());

  std::string baselink_ns = s + "base_link";
  std::string map_ns = "robot_1/map";

  // Stanley tracker;
  Stanley *stanley = new Stanley;
  Kanayama *kanayama = new Kanayama;
  if (tracker_name == "Stanley")
  {
    delete kanayama;
  }
  else if (tracker_name == "Kanayama")
  {
    delete stanley;
  }

  // sub = nh.subscribe("tf",1000,odomCallback);
  sub_trajectory = nh.subscribe(s + "path", 1000, pathCallback);

  pub = nh.advertise<geometry_msgs::TransformStamped>(s + "test_data", 1);
  pub_vel = nh.advertise<geometry_msgs::Twist>(s + "cmd_vel", 1);
  pub_dp = nh.advertise<geometry_msgs::PointStamped>(s + "dp", 1);
  pub_left_traj = nh.advertise<nav_msgs::Path>(s + "left_traj", 1);

  emer_sub = nh.subscribe("/emer_flag", 1000, emer_Callback);

  tf2_ros::TransformListener tfListener(tfBuffer);

  double sT = 0;
  double nT = 0;

  ros::Rate rate(hz);
  while (ros::ok())
  {
    // std::cout << emer_flag;
    tf_Listener(map_ns, baselink_ns);

    if (trajectiory_subscribed)
    {
      if (tracker_name == "Stanley")
      {
        stanley->Set_parameters(k, k2, v, hz);
        stanley->Set_robot_pos(robot_X, robot_Y, robot_Yaw);
        stanley->Set_trajectory(path);
      }
      else if (tracker_name == "Kanayama")
      {
        kanayama->Set_parameters(k_x, k_y, timeStep, hz);
        kanayama->Set_robot_pos(robot_X, robot_Y, robot_Yaw);
        kanayama->Set_trajectory(path);
      }
      trajectiory_subscribed = false;
      start_tracking = true;
      ros::Time startTime = ros::Time::now();
      sT = startTime.sec + startTime.nsec * (1e-9);
    }
    else if (tf_listened && start_tracking && emer_flag)
    {
      ros::Time presentTime = ros::Time::now();
      nT = presentTime.sec + presentTime.nsec * (1e-9) - sT;

      // for visualization about reference pose
      geometry_msgs::PointStamped pnt;
      geometry_msgs::Pose pose_;
      pnt.header.frame_id = "robot_1/map";

      // for visualization about left trajectory
      nav_msgs::Path leftTraj;

      if (tracker_name == "Stanley")
      {
        stanley->Set_robot_pos(robot_X, robot_Y, robot_Yaw, nT);
        cmd_vel = stanley->Get_vel();
        pose_ = stanley->Get_Dp();
        leftTraj = stanley->Left_Traj();
        if (stanley->End_trajectory())
        {
          start_tracking = false;
          ROS_INFO("Tracking Done!!");
        }
      }
      else if (tracker_name == "Kanayama")
      {
        kanayama->Set_robot_pos(robot_X, robot_Y, robot_Yaw, nT);
        cmd_vel = kanayama->Get_vel();
        pose_ = kanayama->Get_Dp();
        leftTraj = kanayama->Left_Traj();
        if (kanayama->End_trajectory())
        {
          start_tracking = false;
          ROS_INFO("Tracking Done!!");
        }
      }

      // pub to rviz
      pnt.point.x = pose_.position.x;
      pnt.point.y = pose_.position.y;
      pub_dp.publish(pnt);
      pub_left_traj.publish(leftTraj);
    }
    else
    {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
    }

    // pub robot velocity
    pub_vel.publish(cmd_vel);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
