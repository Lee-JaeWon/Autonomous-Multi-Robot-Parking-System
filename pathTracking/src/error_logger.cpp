#include <ros/ros.h>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <pathTracking/errorData.h>

#include<iostream>
#include<fstream>
#include<string>

bool tf_listened = false;
bool trajectiory_subscribed = false;
bool start_tracking = false;

int trajectory_length;

nav_msgs::Path path;
tf2_ros::Buffer tfBuffer;

double robot_X;
double robot_Y;
double robot_Yaw;
double distP2R;
double heading_error;

pathTracking::errorData DataArray[500];

void pathCallback(const nav_msgs::Path::ConstPtr& path_)
{
  path = *path_;
  trajectiory_subscribed = true;
  trajectory_length = (int)path.poses.size();
}

void tf_Listener()
{
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map", "base_link",
                               ros::Time(0));
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

void Calc_desired_point()
{
  //calc desire pose, theta, dist
  double x, y, theta;
  double min_dist = std::numeric_limits<double>::infinity();
  for(int i=0;i<trajectory_length;i++)
  {
    ROS_INFO("Working...%d",i);
    x = path.poses[i].pose.position.x;
    y = path.poses[i].pose.position.y;
    double dist_ = sqrt((x-robot_X)*(x-robot_X) + (y-robot_Y)*(y-robot_Y));
    ROS_INFO("dist_ %lf",dist_);
    std::cout <<"dist_  : "<<dist_<<"\n";

    if(min_dist > dist_)
    {
      double a11,a12,a21,a22;

      if(i==trajectory_length-1) {
        //theta = atan2(y-this->trajectory.poses[i-1].pose.position.y, x-this->trajectory.poses[i-1].pose.position.x);
        a11 = x-path.poses[i-1].pose.position.x;
        a12 = y-path.poses[i-1].pose.position.y;
      }
      else {
        //theta = atan2(this->trajectory.poses[i+1].pose.position.y-y, this->trajectory.poses[i+1].pose.position.x-x);
        a11 = path.poses[i+1].pose.position.x-x;
        a12 = path.poses[i+1].pose.position.y-y;
      }

      a21 = robot_X-x;
      a22 = robot_Y-y;

      theta = atan2(a12,a11);

      min_dist = dist_;

      std::cout <<"min_dist  : "<<min_dist<<"\n";
      ROS_INFO("error %lf",min_dist);
    }
  }
  heading_error = theta;
  distP2R = min_dist;
}

void Writer()
{
  std::ofstream writeFile;
  writeFile.open("errorData.txt");

  for(int i=0;i<500;i++)
  {
//    writeFile << DataArray[i].time << " " << DataArray[i].cross_track_error <<"\n";
    writeFile << DataArray[i].cross_track_error <<"\n";
  }

  writeFile.close();
}

int main(int argc, char** argv){

   ros::init(argc, argv, "error_logger");

   ros::NodeHandle nh;

   ros::Subscriber sub_trajectory = nh.subscribe("path",1,pathCallback);
   ros::Publisher pub_error = nh.advertise<pathTracking::errorData>("error_data",1);

   tf2_ros::TransformListener tfListener(tfBuffer);

   double sT=0;
   double nT=0;
   int index=0;
   bool write_once=true;

   ros::Rate rate(20);
   while (ros::ok()){

     tf_Listener();
     if(trajectiory_subscribed)
     {
       trajectiory_subscribed = false;
       start_tracking = true;
       ros::Time startTime = ros::Time::now();
       sT = startTime.sec+startTime.nsec*(1e-9);
     }
     else if(tf_listened&&start_tracking)
     {
       ros::Time presentTime = ros::Time::now();
       nT = presentTime.sec+presentTime.nsec*(1e-9) - sT;

       Calc_desired_point();
       pathTracking::errorData eData;
       eData.stamp = ros::Time::now();
       eData.time = (float)nT;
       eData.heading_error = (float)heading_error;
       eData.cross_track_error = (float)distP2R;
       pub_error.publish(eData);

       if(index<500)
       {
         DataArray[index] = eData;
         index++;
       }
       else if(index>=500 && write_once)
       {
         Writer();
         write_once = false;
         start_tracking = false;
         ROS_INFO("WRITE DONE!");
       }

     }

     ros::spinOnce();
     rate.sleep();
   }
   return 0;
 };


