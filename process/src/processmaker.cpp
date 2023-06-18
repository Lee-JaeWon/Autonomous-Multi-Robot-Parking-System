#include "process/processmaker.h"

void doneMoveCb(const actionlib::SimpleClientGoalState& state,const parking_msgs::parkingOrderResultConstPtr &result)
{
  if(result->sequence)
  {
    std::cout<<"Move result is True"<<"\n";
    int i = result->i;
    int j = result->j;
    int k = result->k;
    int rNum = result->robotNum;
    sequence.SetProcessDone(i, j, k, rNum);

    parking_msgs::parkingDone msg;
    msg.type = sequence.GetSequence().miniSequence[i].order;
    msg.job  = sequence.GetSequence().miniSequence[i].process[j].job;
    msg.parkinglot = sequence.GetSequence().miniSequence[i].process[j].action[k].parkingLot;
    pub_parking_done.publish(msg);
  }
  else {
    std::cout<<"result is False"<<"\n";
  }
}

void activeMoveCb()
{

}

void feedbackMoveCb(const parking_msgs::parkingOrderFeedbackConstPtr & feedback)
{

}

void doneLiftCb(const actionlib::SimpleClientGoalState& state,const parking_msgs::liftOrderResultConstPtr &result)
{
  if(result->sequence)
  {
    std::cout<<"Lift result is True"<<"\n";
    int i = result->i;
    int j = result->j;
    int k = result->k;
    int rNum = result->robotNum;
    sequence.SetProcessDone(i, j, k, rNum);
  }
  else {
    std::cout<<"result is False"<<"\n";
  }
}

void activeLiftCb()
{

}

void feedbackLiftCb(const parking_msgs::liftOrderFeedbackConstPtr & feedback)
{

}

bool OrderCallBack(parking_msgs::order::Request &req, parking_msgs::order::Response &res)
{
  res.accepted = true;
  res.answer = req.carNum;

  //Sequence
  if(req.order==PARKIN)
  {
    sequence.OrderParkIn(req.parkinglot, PL[req.parkinglot], req.carNum);
  }
  else if(req.order==PARKOUT)
  {
    sequence.OrderParkOut(req.parkinglot, PL[req.parkinglot], req.carNum);
  }

  return true;
}

// 0617
void PubAction(std::vector<int> v)
{
  int i = v.at(0);
  int j = v.at(1);
  int k = v.at(2);

  int rNum = seq.miniSequence[i].process[j].robotNumber;
  ROS_INFO("robot %d", rNum);

  std::string type = seq.miniSequence[i].process[j].action[k].action;

  if(type=="Move")
  {
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = seq.miniSequence[i].process[j].action[k].x;
    ps.pose.position.y = seq.miniSequence[i].process[j].action[k].y;

    parking_msgs::parkingOrderGoal goal;
    goal.goal = ps;
    goal.robotNum = rNum;
    goal.i = i;
    goal.j = j;
    goal.k = k;
    goal.rotation = seq.miniSequence[i].process[j].action[k].rotation;

    //ac[rNumber]->waitForServer();
    ac.at(rNum)->sendGoal(goal, &doneMoveCb, &activeMoveCb, &feedbackMoveCb);
  }
  else if(type=="LiftUp")
  {
    parking_msgs::liftOrderGoal goal;
    goal.type = "LiftUp";
    goal.robotNum = rNum;
    goal.i = i;
    goal.j = j;
    goal.k = k;

    acLift.at(rNum)->sendGoal(goal, &doneLiftCb, &activeLiftCb, &feedbackLiftCb);
  }
  else if(type=="LiftDown")
  {
    parking_msgs::liftOrderGoal goal;
    goal.type = "LiftDown";
    goal.robotNum = rNum;
    goal.i = i;
    goal.j = j;
    goal.k = k;

    acLift.at(rNum)->sendGoal(goal, &doneLiftCb, &activeLiftCb, &feedbackLiftCb);
  }
}

void ReadParkingData(int parkingNum)
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
        carList.push_back(true);
      }
      else if(status == "empty")
      {
        carList.push_back(false);
      }

    }
    carList.push_back(false);
    carList.push_back(false);
  }
  readfile.close();
}


int main(int argc, char** argv){

   ros::init(argc, argv, "processmaker");

   ros::NodeHandle nh;
   ros::NodeHandle n("~");

   // Parameters //
   n.getParam("parkingNum",parkingNum);

   PL = new std::vector<double>[parkingNum];
   std::string key = "PL";
   for(int i=0; i<parkingNum; i++)
   {
     std::string paramName = key + std::to_string(i);
     n.getParam(paramName, PL[i]);
   }

   n.getParam("InputSpot", InputSpot);
   n.getParam("OutputSpot", OutputSpot);

   n.getParam("MainSpot",MainSpot);
   n.getParam("robot_num",robot_num);

   // Server //
   server = nh.advertiseService("service",OrderCallBack);

   // Action Client //
   //std::vector<actionlib::SimpleActionClient<parking_msgs::parkingOrderAction>*> ac(robot_num);

   // 각 클라이언트를 초기화 - move
   for (int i = 0; i < robot_num; i++) {
     std::string actionServerName = "robot_" + std::to_string(i+1) + "/action_parking_order";
     ac[i] = new actionlib::SimpleActionClient<parking_msgs::parkingOrderAction>(actionServerName, true);
   }

   // 각 클라이언트를 초기화 -lift
   for (int i = 0; i < robot_num; i++) {
     std::string actionServerName = "robot_" + std::to_string(i+1) + "/action_lift_order";
     acLift[i] = new actionlib::SimpleActionClient<parking_msgs::liftOrderAction>(actionServerName, true);
   }

   // Publisher //
   //pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);
   pub_parking_done = nh.advertise<parking_msgs::parkingDone>("parking_done",1);
   pub_Sequence = nh.advertise<parking_msgs::Sequence>("Sequence",1);
   pubMove = new ros::Publisher[robot_num];
   robot_namespace = new std::string[robot_num];
   std::string baseName = "/robot_";
   for(int i=0; i<robot_num; i++)
   {
     robot_namespace[i] = baseName + std::to_string(i+1);
     std::string topicName = robot_namespace[i] + "/move_base_simple/goal";
     pubMove[i] = nh.advertise<geometry_msgs::PoseStamped>(topicName,1);
   }

   // Init //
   ParkingData = new ParkingInfo[parkingNum];
   ReadParkingData(parkingNum);
   robotList = {0,3,2,0,0,0,1,0}; //PL0~5 + InLot + OutLot
   paletteList = {false, true, true, true, true, true, true, false};

   // Sequence //
   sequence.SetRobotNum(robot_num);
   sequence.SetInOutSpot(InputSpot,OutputSpot);
   sequence.SetLists(carList, paletteList, robotList);
   sequence.SetPL(PL);

   //ac.waitForServer();

   ros::Rate rate(30);
   while (ros::ok()){

     //시퀀스변동사항 있으면
     if(seq!=sequence.GetSequence())
     {
       std::cout<<"Sequence change!"<<"\n";
       seq = sequence.GetSequence();
       pub_Sequence.publish(seq);
       // 스택갱신
       sequence.Seq2Stack();
     }
     else
     {
       seq = sequence.GetSequence();
     }

     //0617 new
     sequence.PopStack();

     for(int i=0;i<robot_num;i++)
     {
       if(sequence.CheckOrderEmpty(i))
       {
         PubAction(sequence.GiveOrder(i));
       }

     }

     for(int i=0; i<seq.miniSequence.size();i++)
     {
       if(seq.miniSequence[i].condition=="Done")
       {
         //sequence.RemoveSequence();
       }
     }

     ros::spinOnce();
     rate.sleep();
   }
   return 0;
};
