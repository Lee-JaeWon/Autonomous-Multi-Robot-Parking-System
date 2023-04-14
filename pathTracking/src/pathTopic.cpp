#include <ros/ros.h>
#include <cmath>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <pathTracking/errorData.h>

#include<iostream>
#include<fstream>
#include<string>

#include <ctime>

ros::Publisher pub_trajectory;

std::string topicMode = "sub";
std::string fileName = "path-4M13D19h58m39s";
std::string filePath = "home/hyedo/catkin_ws/src/pathTracking/pathData/";

nav_msgs::Path Reader(std::string fileName)
{
  std::cout<<fileName<<"\n";
  std::ifstream readile(fileName);
  if(readile.fail())
  {
    std::cerr << "Error!" << std::endl;
  }

  nav_msgs::Path path;

  if (!readile.eof()) {
      int length;
      readile >> length;
      path.poses.resize(length);
      path.header.frame_id="map";
      std::cout<<" length : "<<length<<"\n";
      for(int i=0;i<length;i++)
      {
        double x,y,z;
        readile >> x;
        readile >> y;
        readile >> z;
        path.poses[i].pose.position.x=x;
        path.poses[i].pose.position.y=y;
        path.poses[i].pose.position.z=z;
        std::cout<<" index : "<<i<<"\n";
        std::cout<<" x : "<<path.poses[i].pose.position.x<<"\n";
        std::cout<<" y : "<<path.poses[i].pose.position.y<<"\n";
        std::cout<<" z : "<<path.poses[i].pose.position.z<<"\n";

//        readile >> path.poses[i].pose.position.x;
//        readile >> path.poses[i].pose.position.y;
//        readile >> path.poses[i].pose.position.z;
      }

  }
  readile.close();
  std::cout<<"Read Success!! \n";
  //pub_trajectory.publish(path);
  return path;
}

void Writer(nav_msgs::Path path, int length)
{
  std::ofstream writeFile;

  time_t timer;
  struct tm* t;
  timer = time(NULL);
  t = localtime(&timer);

  std::string date;
  //date = "path_";
  date = filePath;

  date += "path-";

  date += std::to_string(t->tm_mon + 1);
  date += "M";
  date += std::to_string(t->tm_mday);
  date += "D";
  date += std::to_string(t->tm_hour);
  date += "h";
  date += std::to_string(t->tm_min);
  date += "m";
  date += std::to_string(t->tm_sec);
  date += "s";
  date += ".txt";

  writeFile.open(date);

  writeFile << length<<"\n";

  for(int i=0;i<length;i++)
  {
    writeFile << path.poses[i].pose.position.x;
    writeFile << " ";
    writeFile << path.poses[i].pose.position.y;
    writeFile << " ";
    writeFile << path.poses[i].pose.position.z;
    writeFile << "\n";
  }

  std::cout<<"Write Done!\n";
  std::cout<<"File path/name : "<<date;

  writeFile.close();
}

void pathCallback(const nav_msgs::Path::ConstPtr& path_)
{
  if(topicMode=="sub")
  {
    std::cout<<"Received path!\n";
    nav_msgs::Path path;
    path = *path_;

    int trajectory_length;
    trajectory_length = (int)path.poses.size();
    Writer(path,trajectory_length);
  }
}

nav_msgs::Odometry staticOdomPublisher()
{
  nav_msgs::Odometry odom;

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x=-2.00002659708956;
  odom.pose.pose.position.y=-0.49995683370913635;
  odom.pose.pose.position.z=-0.0010013929948064736;

  return odom;
}

int main(int argc, char** argv){

   ros::init(argc, argv, "pathTopic");

   ros::NodeHandle nh;
   ros::NodeHandle n("~");

   n.getParam("topicMode", topicMode);
   n.getParam("fileName", fileName);
   n.getParam("filePath", filePath);

   ros::Subscriber sub_trajectory = nh.subscribe("path",1,pathCallback);
   ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom",1);

   pub_trajectory = nh.advertise<nav_msgs::Path>("path",1000);

   bool Once = true;
   int trigger=0;

   ros::Rate rate(1);
   while (ros::ok()){

     if(topicMode=="sub")
     {
       pub_odom.publish(staticOdomPublisher());
     }
     else if(topicMode=="pub"&&(trigger==2))
     {
       //Reader(filePath+fileName);
       pub_trajectory.publish(Reader(filePath+fileName));
       Once = false;
     }
     trigger++;

     ros::spinOnce();
     rate.sleep();
   }
   return 0;
 };


