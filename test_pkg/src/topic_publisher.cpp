#include "ros/ros.h"// ROS 기본 헤더 파일
#include "test_pkg/MsgTutorial.h"// MsgTutorial 메시지 파일 헤더(빌드 후 자동 생성 됨)
int main(int argc, char **argv) // 노드 메인 함수
{
ros::init(argc, argv, "topic_publisher"); // 노드명 초기화
ros::NodeHandle nh;// ROS 시스템과 통신을 위한 노드 핸들 선언
// 퍼블리셔선언, ros_tutorials_topic패키지의MsgTutorial메시지파일을이용한
// 퍼블리셔ros_tutorial_pub를작성한다. 토픽명은"ros_tutorial_msg" 이며,
// 퍼블리셔큐(queue) 사이즈를100개로설정한다는것이다
ros::Publisher ros_tutorial_pub= nh.advertise<test_pkg::MsgTutorial>("ros_tutorial_msg", 100);
// 루프주기를설정한다. "10" 이라는것은10Hz를말하는것으로0.1초간격으로반복된다
ros::Rate loop_rate(10);
// MsgTutorial메시지파일형식으로msg라는메시지를선언
test_pkg::MsgTutorial msg;
// 메시지에사용될변수선언

int count = 0;

while (ros::ok())
{
msg.stamp= ros::Time::now();// 현재시간을msg의하위stamp 메시지에담는다
msg.data= count; // count라는변수값을msg의하위data 메시지에담는다
ROS_INFO("send msg= %d", msg.stamp.sec); // stamp.sec메시지를표시한다
ROS_INFO("send msg= %d", msg.stamp.nsec); // stamp.nsec메시지를표시한다
ROS_INFO("send msg= %d", msg.data); // data 메시지를표시한다
ros_tutorial_pub.publish(msg);// 메시지를발행한다
loop_rate.sleep();// 위에서정한루프주기에따라슬립에들어간다
++count;// count 변수1씩증가
}

return 0;
}