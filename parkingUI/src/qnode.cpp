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
#include "../include/parkingUI/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace parkingUI {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
  //free(ParkingData);
  //ParkingData = NULL;
  delete[] ParkingData;
    if(ros::isStarted()) {    
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"parkingUI");
  if ( ! ros::master::check() ) {
          return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  // Prameters //
  n.getParam("parkingNum",parkingNum);
  n.getParam("PL_SIZE", PL_SIZE);

  PL = new std::vector<double>[parkingNum];
  std::string key = "PL";
  for(int i=0; i<parkingNum; i++)
  {
    std::string paramName = key + std::to_string(i);
    n.getParam(paramName, PL[i]);
  }

  n.getParam("MainSpot",MainSpot);
  n.getParam("InputSpot",InputSpot);
  n.getParam("OutputSpot",OutputSpot);
  n.getParam("robot_num",robot_num);

  nh.getParam("map/resolution",map_resolution);

  std::string baseName = "/robot_";
  robot_namespace = new std::string[robot_num];
  robotPose_subsciber = new ros::Subscriber[robot_num];
  for(int i=0;i<robot_num;i++)
  {
    robot_namespace[i] = baseName + std::to_string(i+1);
    std::string topicName = robot_namespace[i] + "/odom";
    robotPose_subsciber[i] = nh.subscribe(topicName,1,&QNode::RobotPoseCallback,this);
  }

  // Publisher //
  chatter_publisher = nh.advertise<std_msgs::String>("chatter", 1000);
  parking_goal_publisher = nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);

  // Subscriber //
  //robotPose_subsciber = nh.subscribe("odom",1,&QNode::RobotPoseCallback,this);
  parkingDone_subscriber = nh.subscribe("parking_done",1,&QNode::ParkingDoneCallback,this);
  Sequence_subscriber = nh.subscribe("Sequence",1,&QNode::SequenceCallback, this);

  //Client
  client = nh.serviceClient<parking_msgs::order>("service");
  clientCarNum = nh.serviceClient<parking_msgs::carNum>("carNumSignal");

  // Data Read & into Class //
  //free(ParkingData);
  //ParkingData = (ParkingInfo*)malloc(sizeof(ParkingInfo) * 4);
  ParkingData = new ParkingInfo[parkingNum];
  ReadParkingData(ParkingData, parkingNum);

	start();
	return true;
}

void QNode::print(std::list<int> const &list)
{
    for (auto const &i: list) {
        std::cout << i << std::endl;
    }
}

void QNode::run() {

  ros::Rate loop_rate(10);
	while ( ros::ok() ) {

		ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::ReadParkingData(ParkingInfo* ParkingData, int parkingNum)
{
  std::string filePath = "../catkin_ws/src/parkingUI/config/ParkingLotInfo.txt";

  //std::cout<<filePath<<"\n";
  std::ifstream readfile(filePath);
  if(readfile.fail())
  {
    std::cerr << "Error!" << std::endl;
  }

  if (!readfile.eof()) {

    for(int i=0; i< parkingNum; i++)
    {
      int parkingLot;
      readfile >> parkingLot;

      std::string status;
      readfile >> status;

      if(status == "full")
      {
        std::string carInfo;
        readfile >> carInfo;

        int Year, Month, Date, Hour, Minute;
        readfile >> Year;
        readfile >> Month;
        readfile >> Date;
        readfile >> Hour;
        readfile >> Minute;
        ParkingData[i] = ParkingInfo();
        ParkingData[i].SetDataInit(parkingLot,status,carInfo,Year,Month,Date,Hour,Minute);
      }
      else {
        ParkingData[i] = ParkingInfo();
        ParkingData[i].SetDataInit(parkingLot,status);

        //Push on the list of currently empty parkingLot
        EmptyList.push_back(parkingLot);

      }
    }
  }
  readfile.close();
}

void QNode::WriteParkingData()
{
  std::string filePath = "../catkin_ws/src/parkingUI/config/ParkingLotInfo.txt";

  std::ofstream writefile(filePath,std::ios::trunc);
  if(writefile.is_open())
  {
    std::string s;
    for(int i=0; i<parkingNum;i++)
    {
      writefile << ParkingData[i].GetParkingLot();
      writefile << " ";
      writefile << ParkingData[i].GetParkingStatus();
      if(ParkingData[i].GetParkingStatus()=="full")
      {
        writefile << " ";
        writefile << ParkingData[i].GetCarNum();
        writefile << " ";
        writefile << ParkingData[i].GetYear();
        writefile << " ";
        writefile << ParkingData[i].GetMonth();
        writefile << " ";
        writefile << ParkingData[i].GetDate();
        writefile << " ";
        writefile << ParkingData[i].GetHour();
        writefile << " ";
        writefile << ParkingData[i].GetMinute();
      }
      writefile << "\n";
    }
  }
  writefile.close();

  ReadParkingData(ParkingData, parkingNum);
}

void QNode::RobotPoseCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  Q_EMIT RobotPose_SIGNAL(odom);
}

void QNode::ParkingDoneCallback(const parking_msgs::parkingDone::ConstPtr &data)
{
  Q_EMIT ParkingDone_SIGNAL(data);
}

void QNode::SequenceCallback(const parking_msgs::Sequence::ConstPtr &seq)
{
  Q_EMIT Sequence_SIGNAL(seq);
}

void QNode::ParkingGoalPublsiher(std::vector<double> goal)
{
  geometry_msgs::PointStamped point;
  point.header.frame_id = "map";
  point.point.x = goal.at(0);
  point.point.y = goal.at(1);
  point.point.z = 0.0;

  parking_goal_publisher.publish(point);
}



}  // namespace parkingUI
