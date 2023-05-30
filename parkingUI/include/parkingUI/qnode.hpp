/**
 * @file /include/class1_ros_qt_demo/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef parkingUI_QNODE_HPP_
#define parkingUI_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <stdlib.h>
#include <QThread>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include "src/ParkingInfo.cpp"
#include <parking_msgs/order.h>
#include <geometry_msgs/PoseStamped.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace parkingUI {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
  bool init();
	void run();

Q_SIGNALS:
  void rosShutdown();
  void RobotPose_SIGNAL(nav_msgs::Odometry::ConstPtr odom);
  void RobotPose1_SIGNAL(geometry_msgs::PoseStamped::ConstPtr oodm);
  void ParkingDone_SIGNAL(std_msgs::Bool::ConstPtr data);

private:
	int init_argc;
	char** init_argv;

  //Publisher
  ros::Publisher chatter_publisher;
  ros::Publisher parking_goal_publisher;


  //Subscriber
  ros::Subscriber robotPose_subsciber;
  ros::Subscriber robotPose_subsciber1;
  ros::Subscriber parkingDone_subscriber;

public:
  //Client
  ros::ServiceClient client;


  // Parameters
public:
  // Map data
  double map_resolution=0.05;

  // <Parking-Lot Coordinates >
  std::vector<double> PL[4];
  int parkingNum;
  std::vector<double> MainSpot;

  //ParkingLot Data
  ParkingInfo* ParkingData=NULL;
  std::list<int> EmptyList;
  void ReadParkingData(ParkingInfo* ParkingData, int parkingNum);
  void WriteParkingData();

  //Callback Function
  void RobotPoseCallback(const nav_msgs::Odometry::ConstPtr &odom);
  void RobotPoseCallback1(const geometry_msgs::PoseStamped::ConstPtr &odom);
  void ParkingDoneCallback(const std_msgs::Bool::ConstPtr &data);

  //Publish Function
  void ParkingGoalPublsiher(std::vector<double> goal);

  void print(std::list<int> const &list);


};

}  // namespace parkingUI

#endif /* parkingUI_QNODE_HPP_ */
