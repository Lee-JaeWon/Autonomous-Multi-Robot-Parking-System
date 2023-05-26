#include <ros/ros.h>
#include <cstdlib>
#include "../../process/src/Sequence.cpp"
#include <parking_msgs/order.h>
#include <parking_msgs/action.h>
#include <parking_msgs/robotProcess.h>
#include <parking_msgs/miniSequence.h>
#include <parking_msgs/Sequence.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "../../../parkingUI/src/ParkingInfo.cpp"
#include <vector>
#include <iostream>

//Publishers
ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub_Sequence;

//Subscribers
ros::Subscriber sub;

ros::ServiceServer server;

//variables
ParkingInfo* ParkingData=NULL;
std::vector<double> PL[4];
int parkingNum;
std::vector<double> MainSpot;

parking_msgs::Sequence seq;

enum {PARKIN, PARKOUT};

//Sequence
Sequence sequence;

//Methods
bool OrderCallBack(parking_msgs::order::Request &req, parking_msgs::order::Response &res);
void ParkingGoalPublsiher(std::vector<double> goal);
