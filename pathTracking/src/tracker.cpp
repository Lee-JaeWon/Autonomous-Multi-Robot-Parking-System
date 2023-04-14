#include "pathTracking/tracker.h"

#include "stanley.cpp"

#include "kanayama.cpp"

#include <pathTracking/errorData.h>


void pathCallback(const nav_msgs::Path::ConstPtr& path_)
{
  path = *path_;
  trajectiory_subscribed = true;
  //pub.publish(path_);
}

void tf_Listener()
{
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "base_link",
                               ros::Time(0));
      //pub.publish(transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      tf_listened = false;
    //    ros::Duration(1.0).sleep();
    //    continue;
      return;
    }

    tf_listened = true;

    //Find heading angle of robot
    tf2::Quaternion quaternion(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w);

    double roll, pitch, yaw;

    tf2::Matrix3x3 m(quaternion);
    m.getRPY(roll, pitch, yaw);

    //getting from x and y coordinate of robot in global frame
    robot_X = transformStamped.transform.translation.x;
    robot_Y = transformStamped.transform.translation.y;
    robot_Yaw = yaw;
}

int main(int argc, char** argv){

   ros::init(argc, argv, "tracker");

   ros::NodeHandle nh;
   ros::NodeHandle n("~");

   //get Parameters
   n.getParam("hz", hz);
   n.getParam("tracker", tracker_name);

   //Param - Stanley
   n.getParam("k", k);
   n.getParam("k2", k2);
   n.getParam("v", v);

   //Param - Kanayama
   n.getParam("k_x", k_x);
   n.getParam("k_y", k_y);
   n.getParam("timeStep", timeStep);

   //Stanley tracker;
   Stanley* stanley = new Stanley;
   Kanayama* kanayama = new Kanayama;
   if(tracker_name == "Stanley")
   {
     delete kanayama;
   }
   else if(tracker_name == "Kanayama")
   {
     delete stanley;
   }

   //sub = nh.subscribe("tf",1000,odomCallback);
   sub_trajectory = nh.subscribe("path",1,pathCallback);
   //pub = nh.advertise<nav_msgs::Path>("path_",1);
   pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
   pub_dp = nh.advertise<geometry_msgs::PointStamped>("dp",1);
   pub_left_traj = nh.advertise<nav_msgs::Path>("left_traj",1);

   tf2_ros::TransformListener tfListener(tfBuffer);

   double sT=0;
   double nT=0;

   int errCnt=0;
   double distErr=0;
   double meanErr=0;

   ros::Rate rate(hz);
   while (ros::ok()){

     tf_Listener();

     if(trajectiory_subscribed)
     {
       if(tracker_name == "Stanley")
       {
         stanley->Set_parameters(k, k2, v, hz);
         stanley->Set_robot_pos(robot_X,robot_Y,robot_Yaw);
         stanley->Set_trajectory(path);
       }
       else if(tracker_name == "Kanayama")
       {
         kanayama->Set_parameters(k_x, k_y, timeStep, hz);
         kanayama->Set_robot_pos(robot_X,robot_Y,robot_Yaw);
         kanayama->Set_trajectory(path);
       }
       trajectiory_subscribed = false;
       start_tracking = true;
       ros::Time startTime = ros::Time::now();
       sT = startTime.sec+startTime.nsec*(1e-9);
     }
     else if(tf_listened&&start_tracking)
     {
       ros::Time presentTime = ros::Time::now();
       nT = presentTime.sec+presentTime.nsec*(1e-9) - sT;

       //for visualization about reference pose
       geometry_msgs::PointStamped pnt;
       geometry_msgs::Pose pose_;
       pnt.header.frame_id = "map";

       //for visualization about left trajectory
       nav_msgs::Path leftTraj;

       if(tracker_name == "Stanley")
       {
         stanley->Set_robot_pos(robot_X,robot_Y,robot_Yaw, nT);
         cmd_vel = stanley->Get_vel();
         pose_ = stanley->Get_Dp();
         leftTraj = stanley->Left_Traj();

         errCnt++;
         distErr+=stanley->return_Dist();

         if(stanley->End_trajectory())
         {
           start_tracking = false;
           ROS_INFO("Tracking Done!!");
           meanErr = distErr/errCnt;
           ROS_INFO("mean error : %f",meanErr);
         }
       }
       else if(tracker_name == "Kanayama")
       {
         kanayama->Set_robot_pos(robot_X,robot_Y,robot_Yaw, nT);
         cmd_vel = kanayama->Get_vel();
         pose_ = kanayama->Get_Dp();
         leftTraj = kanayama->Left_Traj();

         errCnt++;
         distErr+=kanayama->return_Dist();

         if(kanayama->End_trajectory())
         {
           start_tracking = false;
           ROS_INFO("Tracking Done!!");
           meanErr = distErr/errCnt;
           ROS_INFO("mean error : %f",meanErr);
         }
       }

       //pub to rviz
       pnt.point.x= pose_.position.x;
       pnt.point.y= pose_.position.y;
       pub_dp.publish(pnt);
       pub_left_traj.publish(leftTraj);

     }
     else {
       cmd_vel.linear.x=0.0;
       cmd_vel.angular.z=0.0;
     }

     //pub robot velocity
     pub_vel.publish(cmd_vel);

     ros::spinOnce();
     rate.sleep();
   }
   return 0;
 };


