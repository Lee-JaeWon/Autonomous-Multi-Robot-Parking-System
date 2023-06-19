#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Bool.h>

// For Action
#include "actionlib/server/simple_action_server.h"
#include <parking_msgs/Planner2TrackerAction.h>


//Messages
nav_msgs::Odometry  state;
nav_msgs::Path global_path;
nav_msgs::Path local_path;
geometry_msgs::TransformStamped transformStamped;
geometry_msgs::Twist cmd_vel;

//tf2
tf2_ros::Buffer tfBuffer;

//Publishers
ros::Publisher pub;
ros::Publisher pub_vel;
ros::Publisher pub_vel_pt;
ros::Publisher fake_pub_vel;

//Publishers-rviz
ros::Publisher pub_dp;
ros::Publisher pub_left_traj;

//Subscribers
ros::Subscriber sub;
ros::Subscriber sub_global_trajectory;
ros::Subscriber sub_local_trajectory;

ros::Subscriber emer_sub;
ros::Subscriber vel_stop_sub;

//variables
bool tf_listened = false;
bool trajectiory_subscribed = false;
bool start_tracking = false;

std::string s; //for namespace

double distError;
double robot_X;
double robot_Y;
double robot_Yaw;

geometry_msgs::Twist ref_vel;

//Hz Paramaeter
int hz = 33;
std::string tracker_name = "Stanley";

//Stanley Parameters
double k = 4.0;
double k2 = 1.5;
double v = 0.22;

//Kanayama Parameters
double k_x = 0;
double k_y = 0;
double k_theta = 0;
double timeStep = 0.3;

//Stanley + PID Parameters
double ph = 1.8;
double ih = 0.0;
double dh = 1.3;
//
double pc = 1.8;
double ic = 0.0;
double dc = 1.5;

//emergency
bool emer_flag = true;
bool velstop_flag = true;


void pathCallback(const nav_msgs::Path::ConstPtr& path_);
void tf_Listener();
void accelerator(geometry_msgs::Twist cmd_vel)
{
//  if(ref_vel.linear.x != 0)
//  {
//    ref_vel.angular.z = ref_vel.angular.z + (cmd_vel.linear.z - ref_vel.angular.z)/20;
//  }
//  else {
//    ref_vel = cmd_vel;
//  }

  ref_vel.linear.x = ref_vel.linear.x + (cmd_vel.linear.x - ref_vel.linear.x)/50;
  ref_vel.angular.z = ref_vel.angular.z + (cmd_vel.linear.z - ref_vel.angular.z)/50;
}


// Action Server Class - Planner2Tracker
class Planner2TrackerAction {

protected:
  // 노드 핸들
  ros::NodeHandle nh_;
  // 액션 서버
  actionlib::SimpleActionServer<parking_msgs::Planner2TrackerAction> as_;
  // 액션 이름
  std::string action_name_;
  // 액션 피드백 및 결과
//
  parking_msgs::Planner2TrackerResult result_;
  parking_msgs::Planner2TrackerFeedback feedback_;
public:
  // 액션 서버 초기화
  //bool start=false;
  bool isSuccess = false;
  bool action_called = false;

  //goal 변수
  parking_msgs::Planner2TrackerGoal goal_;

  Planner2TrackerAction(std::string name) :
  // 핸들, 액션 이름, 콜백, auto_start
  // actionlib document에 따르면 auto_start가 true일때 ros::spin을 사용하지 않아도 된다고 합니다.
  // boost::bind는 메소드를 callback으로 넘겨주며,
  // 전달되는 첫 번째 값을 callback 함수의 2번째 파라미터로 넘겨줍니다. (첫 번째는 자기자신)
  as_(nh_, name, boost::bind(&Planner2TrackerAction::executeCB, this, _1), false),
  action_name_(name) {
    as_.start();
  }

  ~Planner2TrackerAction() {
  }

  // goal을 받아 지정된 액션을 수행
  void executeCB(const parking_msgs::Planner2TrackerGoalConstPtr &goal) {
    ROS_INFO("plan to tracker action called");
    this->action_called = true;
    this->goal_ = *goal;
    //goal_.rotation=true;
    

    //글로벌 변수로 전달
    local_path = goal_.path;
    trajectiory_subscribed = true;


    while(!isSuccess) //action not done = still running
    {
      if((int)ros::Time::now().toSec() % 5 == 0)
      {
        int l = goal->path.poses.size();
        ROS_INFO("Path x : %f,  y : %f", goal->path.poses[l].pose.position.x, goal->path.poses[l].pose.position.y);
        feedback_.percent = 0;
        as_.publishFeedback(feedback_);
      }

    }

    if(isSuccess) //action done
    {
      isSuccess = false;
      result_.result=true;
      result_.robotNum=goal_.robotNum;
      as_.setSucceeded(result_);
    }
  }

  const nav_msgs::Path getTarget()
  {

    return this->goal_.path;
  }
};


