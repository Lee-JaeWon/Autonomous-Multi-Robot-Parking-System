#include "process/processmaker.h"

void doneCb(const actionlib::SimpleClientGoalState &state, const parking_msgs::parkingOrderResultConstPtr &result)
{
  if (result->sequence)
  {
    ROS_INFO("4");
    std::cout << "result is True"
              << "\n";
    int i = result->i;
    int j = result->j;
    int k = result->k;
    parking_msgs::parkingDone msg;
    msg.type = sequence.GetSequence().miniSequence[i].order;
    msg.job = sequence.GetSequence().miniSequence[i].process[j].job;
    msg.parkinglot = sequence.GetSequence().miniSequence[i].process[j].action[k].parkingLot;
    pub_parking_done.publish(msg);
    sequence.SetProcessDone(i, j, k);
  }
  else
  {
    std::cout << "result is False"
              << "\n";
  }
}

bool OrderCallBack(parking_msgs::order::Request &req, parking_msgs::order::Response &res)
{
  res.accepted = true;
  res.answer = req.carNum;

  // Sequence
  if (req.order == PARKIN)
  {
    sequence.OrderParkIn(req.parkinglot, PL[req.parkinglot], req.carNum);
  }
  else if (req.order == PARKOUT)
  {
    sequence.OrderParkOut(req.parkinglot, req.carNum);
  }

  // ParkingGoalPublsiher(PL[(int)req.parkinglot]);

  return true;
}

void ParkingGoalPublsiher(std::vector<double> goal)
{
  geometry_msgs::PoseStamped point;
  point.header.frame_id = "map";

  point.pose.position.x = goal.at(0);
  point.pose.position.y = goal.at(1);
  point.pose.position.z = 0.0;

  // pub.publish(point);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "processmaker");

  ros::NodeHandle nh;
  ros::NodeHandle n("~");

  // Parameters //
  n.getParam("parkingNum", parkingNum);

  PL = new std::vector<double>[parkingNum];
  std::string key = "PL";
  for (int i = 0; i < parkingNum; i++)
  {
    std::string paramName = key + std::to_string(i);
    n.getParam(paramName, PL[i]);
  }

  n.getParam("InputSpot", InputSpot);
  n.getParam("OutputSpot", OutputSpot);

  n.getParam("LiftingSpot", LiftingSpot);
  n.getParam("LiftingSpot_two", LiftingSpot_two);
  n.getParam("LiftingSpot_thr", LiftingSpot_thr);

  n.getParam("MainSpot", MainSpot);
  n.getParam("robot_num", robot_num);

  // Server //
  server = nh.advertiseService("service", OrderCallBack);

  // Action Client //
  std::vector<actionlib::SimpleActionClient<parking_msgs::parkingOrderAction> *> ac(robot_num);

  // 각 클라이언트를 초기화
  for (int i = 0; i < robot_num; i++)
  {
    std::string actionServerName = "robot_" + std::to_string(i + 1) + "/action_parking_order";
    ac[i] = new actionlib::SimpleActionClient<parking_msgs::parkingOrderAction>(actionServerName, true);
  }

  // Publisher //
  // pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);
  pub_parking_done = nh.advertise<parking_msgs::parkingDone>("parking_done", 1);
  pub_Sequence = nh.advertise<parking_msgs::Sequence>("Sequence", 1);
  pubMove = new ros::Publisher[robot_num];
  robot_namespace = new std::string[robot_num];
  std::string baseName = "/robot_";
  for (int i = 0; i < robot_num; i++)
  {
    robot_namespace[i] = baseName + std::to_string(i + 1);
    std::string topicName = robot_namespace[i] + "/move_base_simple/goal";
    pubMove[i] = nh.advertise<geometry_msgs::PoseStamped>(topicName, 1);
  }

  // Init //
  ParkingData = new ParkingInfo[parkingNum];

  // Sequence //
  sequence.SetInOutSpot(InputSpot, OutputSpot);
  sequence.SetLiftingSpot(LiftingSpot, LiftingSpot_two, LiftingSpot_thr);

  // ac.waitForServer();

  ros::Rate rate(30);
  while (ros::ok())
  {

    if (seq != sequence.GetSequence())
    {
      std::cout << "Sequence change!"
                << "\n";
      seq = sequence.GetSequence();
      pub_Sequence.publish(seq);
    }
    else
    {
      seq = sequence.GetSequence();
    }

    if (sequence.IsHaveOrder() == "Move")
    {

      int robotNumber = sequence.getRobotNum();
      geometry_msgs::PoseStamped stamp = sequence.Mover();
      std::vector<int> vec = sequence.getLocation();

      parking_msgs::parkingOrderGoal goal;
      goal.goal = stamp;
      goal.robotNum = robotNumber;
      goal.i = vec.at(0);
      goal.j = vec.at(1);
      goal.k = vec.at(2);

      // ac[rNumber]->waitForServer();
      ac.at(robotNumber)->sendGoal(goal, &doneCb);
      // ac.at(robotNumber)->sendGoal(goal);
    }
    else if (sequence.IsHaveOrder() == "LiftUp")
    {
    }
    else if (sequence.IsHaveOrder() == "LiftDown")
    {
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
