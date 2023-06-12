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
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/terminal_state.h"
#include <parking_msgs/parkingOrderAction.h>
#include <parking_msgs/parkingDone.h>


//Publishers
ros::Publisher pub;
ros::Publisher pub_Sequence;
ros::Publisher* pubMove; //가야하는 point 찍어주는 퍼블리셔
ros::Publisher pub_parking_done;

//Subscribers
ros::Subscriber sub;

//Service Server
ros::ServiceServer server;

//Action Client


//variables
ParkingInfo* ParkingData=NULL;
std::vector<double>* PL;
int parkingNum;
std::vector<double> MainSpot;
std::vector<double> InputSpot;
std::vector<double> OutputSpot;
int robot_num=3;
std::string* robot_namespace;
//
std::vector<double> LiftingSpot;
std::vector<double> LiftingSpot_two;
std::vector<double> LiftingSpot_thr;

parking_msgs::Sequence seq;

enum {PARKIN, PARKOUT};

//Sequence
Sequence sequence;

//Methods
bool OrderCallBack(parking_msgs::order::Request &req, parking_msgs::order::Response &res);
void ParkingGoalPublsiher(std::vector<double> goal);
