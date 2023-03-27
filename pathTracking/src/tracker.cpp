#include "pathTracking/tracker.h"
#include "stanley.cpp"


void pathCallback(const nav_msgs::Path::ConstPtr& path_)
{
  path = *path_;
  trajectiory_subscribed = true;
}

void tf_Listener()
{
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "base_link",
                               ros::Time(0));
      pub.publish(transformStamped);
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
   n.getParam("k",k);
   n.getParam("k2",k2);
   n.getParam("v",v);


   Stanley tracker;

   //sub = nh.subscribe("tf",1000,odomCallback);
   sub_trajectory = nh.subscribe("path",1000,pathCallback);
   pub = nh.advertise<geometry_msgs::TransformStamped>("test_data",1);
   pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
   pub_dp = nh.advertise<geometry_msgs::PointStamped>("dp",1);

   tf2_ros::TransformListener tfListener(tfBuffer);

   ros::Rate rate(hz);
   while (ros::ok()){

     tf_Listener();

     if(trajectiory_subscribed)
     {
       tracker.Set_parameters(k, k2, v, hz);
       tracker.Set_trajectory(path);
       trajectiory_subscribed = false;
       start_tracking = true;
     }
     else if(tf_listened&&start_tracking)
     {
       tracker.Set_robot_pos(robot_X,robot_Y,robot_Yaw);
       cmd_vel = tracker.Get_vel();
       geometry_msgs::PointStamped pnt;
       geometry_msgs::Pose pose_;
       pnt.header.frame_id = "map";
       pose_ = tracker.Get_Dp();
       pnt.point.x= pose_.position.x;
       pnt.point.y= pose_.position.y;
       pub_dp.publish(pnt);
       if(tracker.End_trajectory())
       {
         start_tracking = false;
         ROS_INFO("Tracking Done!!");
       }
     }
     else {
       cmd_vel.linear.x=0.0;
       cmd_vel.angular.z=0.0;
     }
     pub_vel.publish(cmd_vel);


     //ROS_INFO("yaw : %g", robot_Yaw);


     ros::spinOnce();
     rate.sleep();
   }
   return 0;
 };


