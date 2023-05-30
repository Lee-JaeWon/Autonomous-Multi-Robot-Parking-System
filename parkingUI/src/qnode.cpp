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

  // Prameters //
  ros::NodeHandle n("~");
  n.getParam("parkingNum",parkingNum);

  n.getParam("PL0", PL[0]);
  n.getParam("PL1", PL[1]);
  n.getParam("PL2", PL[2]);
  n.getParam("PL3", PL[3]);

  n.getParam("MainSpot",MainSpot);

  ros::NodeHandle nh;
  // Publisher //
  chatter_publisher = nh.advertise<std_msgs::String>("chatter", 1000);
  parking_goal_publisher = nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);

  // Subscriber //
  robotPose_subsciber = nh.subscribe("odom",1,&QNode::RobotPoseCallback,this);
  robotPose_subsciber1 = nh.subscribe("/robot_2/tracked_pose",1,&QNode::RobotPoseCallback1,this);
  
  parkingDone_subscriber = nh.subscribe("parking_done",1,&QNode::ParkingDoneCallback,this);

  //Client
  client = nh.serviceClient<parking_msgs::order>("service");

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
  std::string filePath = "../catkin_ws/src/Autonomous-Multi-Robot-Parking-System/parkingUI/config/ParkingLotInfo.txt";

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
        //EmptyList->push_back(parkingLot);
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

}

void QNode::RobotPoseCallback(const nav_msgs::Odometry::ConstPtr &odom)
{
  Q_EMIT RobotPose_SIGNAL(odom);
}

void QNode::RobotPoseCallback1(const geometry_msgs::PoseStamped::ConstPtr &odom)
{
  Q_EMIT RobotPose1_SIGNAL(odom);
}

void QNode::ParkingDoneCallback(const std_msgs::Bool::ConstPtr &data)
{
  Q_EMIT ParkingDone_SIGNAL(data);
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
