#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
// #include <opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"

// For Action
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include <parking_msgs/parkingOrderAction.h>
#include <parking_msgs/Planner2TrackerAction.h>

using namespace cv;
using namespace std;

//-------------------------------- Global variables ---------------------------------//
// Subscriber
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber odom_sub;

// Publisher
ros::Publisher path_pub;

// Action Client
actionlib::SimpleActionClient<parking_msgs::Planner2TrackerAction>* ac_plan2track;

// Object
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;
geometry_msgs::PoseStamped path_point;

// Parameter
double InflateRadius;
int rate;
int origin_x, origin_y;
double resol;
int mx, my;
double present_x, present_y;
int robot_num=3;

// Flags
bool start_flag;
bool odomFlag;
bool mapFlag;
bool targetFlag;
bool is_parking_order_result_true=false;

// Triggers
bool action_called=false;

//------------------------------------Action Class------------------------------------//
class parkingOrderAction {

protected:
  // 노드 핸들
  ros::NodeHandle nh_;
  // 액션 서버
  actionlib::SimpleActionServer<parking_msgs::parkingOrderAction> as_;
  // 액션 이름
  std::string action_name_;
  // 액션 피드백 및 결과
  parking_msgs::parkingOrderFeedback feedback_;
  parking_msgs::parkingOrderResult result_;
public:
  // 액션 서버 초기화
  //bool start=false;
  bool isSuccess = false;

  //goal 변수
  parking_msgs::parkingOrderGoal goal_;

  parkingOrderAction(std::string name) :
  // 핸들, 액션 이름, 콜백, auto_start
  // actionlib document에 따르면 auto_start가 true일때 ros::spin을 사용하지 않아도 된다고 합니다.
  // boost::bind는 메소드를 callback으로 넘겨주며,
  // 전달되는 첫 번째 값을 callback 함수의 2번째 파라미터로 넘겨줍니다. (첫 번째는 자기자신)
  as_(nh_, name, boost::bind(&parkingOrderAction::executeCB, this, _1), false),
  action_name_(name) {
    as_.start();
  }

  ~parkingOrderAction() {
  }

  // goal을 받아 지정된 액션을 수행
  void executeCB(const parking_msgs::parkingOrderGoalConstPtr &goal) {
    cout<<"action called12"<<"\n";
    ROS_INFO("action called!");
    action_called = true;
    goal_ = *goal;

    // 시퀀스 vector를 초기화합니다.
    feedback_.percent.push_back(0);
    //start=true;

    for (int i=1; i<=goal->robotNum; i++) {
      // cancel action
      if (as_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
        isSuccess = false;
        break;
      }
      feedback_.percent.push_back(i);
      as_.publishFeedback(feedback_);
    }

    while(!isSuccess) //action not done = still running
    {

    }

    if(isSuccess) //action done
    {
      isSuccess = false;
      result_.sequence = true;
      result_.i=goal_.i;
      result_.j=goal_.j;
      result_.k=goal_.k;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      as_.setSucceeded(result_);
      result_.sequence = false;
    }
  }

  const geometry_msgs::PoseStamped getTarget()
  {
    return goal_.goal;
  }
};

//-------------------------------- Callback function ---------------------------------//
void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    // Get parameter
    resol = msg.info.resolution;
    origin_x = msg.info.origin.position.x;
    origin_y = msg.info.origin.position.y;
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height - i - 1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);
    // Set flag
    mapFlag = true;
    ROS_INFO("map Done");
}

void TargetPointtCallback(const geometry_msgs::PoseStamped &msg)
{
  Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
  OccGridParam.Map2ImageTransform(src_point, targetPoint);
}
void odomCallback(const nav_msgs::Odometry &msg)
{
   mx = int((msg.pose.pose.position.x - origin_x) / (resol)); // world to map
   my = int((msg.pose.pose.position.y - origin_y) / (resol)); // world to map
   Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
   OccGridParam.Map2ImageTransform(src_point, startPoint);

   // Set flag
   odomFlag = true;
}

void done_plan_to_track_Cb(const actionlib::SimpleClientGoalState& state, const parking_msgs::Planner2TrackerResultConstPtr &result)
{
  if(result->result)
  {
    is_parking_order_result_true=true;
  }
  std::cout << "done_plan_to_track_Cb result is : " << result->result << "\n";
}

void publish_path(std::string map_ns, parking_msgs::parkingOrderGoal goal_)
{
  double start_time = ros::Time::now().toSec();
  // Start planning path
  vector<Point> PathList;
  astar.PathPlanning(startPoint, targetPoint, PathList);
  if (!PathList.empty())
  {
      path.header.stamp = ros::Time::now();
      path.header.frame_id = map_ns;
      path.poses.clear();
      for (int i = 0; i < PathList.size(); i++)
      {
          Point2d dst_point;
          OccGridParam.Image2MapTransform(PathList[i], dst_point);

          geometry_msgs::PoseStamped pose_stamped;
          pose_stamped.header.stamp = ros::Time::now();
          pose_stamped.header.frame_id = map_ns;
          pose_stamped.pose.position.x = dst_point.x;
          pose_stamped.pose.position.y = dst_point.y;
          pose_stamped.pose.position.z = 0;
          path.poses.push_back(pose_stamped);
      }
      path_pub.publish(path);

      parking_msgs::Planner2TrackerGoal goal;
      goal.path = path;
      goal.robotNum = goal_.robotNum;
      ac_plan2track->sendGoal(goal, &done_plan_to_track_Cb);

      double end_time = ros::Time::now().toSec();

      ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
  }
  else
  {
      ROS_ERROR("Can not find a valid path");


  }
  int i = 0;

  if ((path_point.pose.position.x - present_x) + (path_point.pose.position.y - present_y) > 0.3)
      i++;

  ROS_INFO("%d\n", i);
  path_point = path.poses[20];
  path_point.pose.orientation.w = 1;
  path_point.pose.orientation.x = 0;
  path_point.pose.orientation.y = 0;
  path_point.pose.orientation.z = 0;
}


//-------------------------------- Main function ---------------------------------//
int main(int argc, char** argv)
{
    //  Initial node
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");

    // param check
    std::string s;
    if (ros::param::get("~namespace", s))
        ROS_INFO("Astar node got param: %s", s.c_str());
    else
        ROS_ERROR("Astar Failed to get param '%s'", s.c_str());

    // Initial variables
    mapFlag = false;
    odomFlag = false;
    targetFlag = false;
    start_flag = false;

    // Parameter
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);
    nh_priv.param<int>("robot_num", robot_num);

    // Subscribe topics
    std::string map_ns = "map"; // fixed frame
    std::string goal_ns = s + "/move_base_simple/goal";
    std::string track_ns = s + "/odom";
    std::string initpose_ns = s + "/initialpose";

    // Subscriber
    map_sub = nh.subscribe((map_ns), 10, MapCallback);
    odom_sub = nh.subscribe(track_ns, 10, odomCallback);

    // Publish topics
    std::string path_ns = s + "/path";

    // Publisher
    path_pub = nh.advertise<nav_msgs::Path>(path_ns, 10);

    // Action Service Class 할당
    std::string action_parking_order_ns = s + "/action_parking_order";
    parkingOrderAction action(action_parking_order_ns);

    // Action Client
    std::string action_plan_to_track_ns = s + "/action_plan_to_track";
    //actionlib::SimpleActionClient<parking_msgs::Planner2TrackerAction> ac_plan2track(action_plan_to_track_ns, true);
    ac_plan2track = new actionlib::SimpleActionClient<parking_msgs::Planner2TrackerAction>(action_plan_to_track_ns, true);

    // Loop and wait for callback
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
      // Call Action Once
      if(action_called && odomFlag && mapFlag)
      {
        TargetPointtCallback(action.getTarget());
        publish_path(map_ns, action.goal_);
        action_called = false;
        //action.isSuccess=true;
      }

      // Wait for end result
      if(is_parking_order_result_true)
      {
        action.isSuccess=true;
        is_parking_order_result_true = false;
      }

      ros::spinOnce();
      loop_rate.sleep();
    }

    return 0;
}

