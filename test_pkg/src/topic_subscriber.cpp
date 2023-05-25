#include "ros/ros.h" // ROS 기본헤더파일
#include "test_pkg/MsgTutorial.h" // MsgTutorial메시지파일헤더(빌드후자동생성됨)
#include "test_pkg/car_data.h"

#include "turtlesim/Kill.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"

void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
void pose_test_method(const geometry_msgs::Pose& test_msg);
void test_method(const test_pkg::MsgTutorial::ConstPtr& t);
void msgCallback(const test_pkg::MsgTutorial::ConstPtr& msg);
void pose_test_method2(const geometry_msgs::Pose& ttt);
// 메시지콜백함수로써, 밑에서설정한ros_tutorial_msg라는이름의토픽
// 메시지를수신하였을때동작하는함수이다
// 입력메시지로는ros_tutorials_topic패키지의MsgTutorial메시지를받도록되어있다
test_pkg::MsgTutorial test;


void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // geometry_msgs::Pose* test_pose;
    ROS_INFO("check");
    geometry_msgs::PoseWithCovarianceStamped test_pose;
    test_pose = *msg;
    pose_test_method(test_pose.pose.pose);
}

void pose_test_method(const geometry_msgs::Pose& test_msg)
{
    ROS_INFO("hello_world!");
    ROS_INFO("%f",test_msg.position.x);
    pose_test_method2(test_msg);
}

void pose_test_method2(const geometry_msgs::Pose& ttt)
{
    ROS_INFO("%f",ttt.position.x);
}
void test_method(const test_pkg::MsgTutorial::ConstPtr& t)
{
    ROS_INFO("hello world");

}
void msgCallback(const test_pkg::MsgTutorial::ConstPtr& msg)
// void msgCallback(const test_pkg::MsgTutorial msg)

{
    // ROS_INFO("recievemsg= %d", msg->stamp.sec); // stamp.sec메시지를표시한다
    // ROS_INFO("recievemsg= %d", msg->stamp.nsec); // stamp.nsec메시지를표시한다
    // ROS_INFO("recievemsg= %d", msg->data); // data 메시지를표시한다
    test = *msg;
    test_method(msg);
    ROS_INFO("%d",msg->data);

    // std::cout << test.data<< std::endl;
}
void CarCB(const test_pkg::car_data::ConstPtr& msg){
    std::cout<< msg->car_number_plate << std::endl;
}
void PathCB(const nav_msgs::Path::ConstPtr& msg){
    for ( auto x : msg->poses){
        
    }

}
int main(int argc, char **argv)// 노드메인함수
{

ros::init(argc, argv, "topic_subscriber");// 노드명초기화
ros::NodeHandle nh; // ROS 시스템과통신을위한노드핸들선언
// 서브스크라이버선언, ros_tutorials_topic패키지의MsgTutorial메시지파일을이용한
// 서브스크라이버ros_tutorial_sub를작성한다. 토픽명은"ros_tutorial_msg" 이며,
// 서브스크라이버큐(queue) 사이즈를100개로설정한다는것이다
ros::Subscriber ros_tutorial_sub= nh.subscribe("ros_tutorial_msg",100, msgCallback);
ros::Subscriber pose_sub= nh.subscribe("/initialpose", 100, PoseCallback);
ros::Subscriber car_sub = nh.subscribe("car_data",100,CarCB);
ros::Subscriber path = nh.subscribe("path",100,PathCB);

// 콜백함수호출을위한함수로써, 메시지가수신되기를대기, 
// 수신되었을경우콜백함수를실행한다
ros::spin();
return 0;

}