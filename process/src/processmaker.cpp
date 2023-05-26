#include "process/processmaker.h"

int main(int argc, char** argv){

   ros::init(argc, argv, "processmaker");

   ros::NodeHandle nh;
   ros::NodeHandle n("~");

   // Parameters //
   n.getParam("parkingNum",parkingNum);

   n.getParam("PL0", PL[0]);
   n.getParam("PL1", PL[1]);
   n.getParam("PL2", PL[2]);
   n.getParam("PL3", PL[3]);

   n.getParam("MainSpot",MainSpot);

   // Server //
   server = nh.advertiseService("service",OrderCallBack);

   // Publisher //
   pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point",1);
   pub1 = nh.advertise<geometry_msgs::PoseStamped>("/robot_2/move_base_simple/goal",1);

   pub_Sequence = nh.advertise<parking_msgs::Sequence>("Sequence",1);

   // Init //
   ParkingData = new ParkingInfo[parkingNum];

   // Sequence //
   //sequence = new Sequence();



   ros::Rate rate(10);
   while (ros::ok()){

     if(seq!=sequence.GetSequence())
     {
       seq = sequence.GetSequence();
       pub_Sequence.publish(seq);

       if(sequence.IsHaveOrder()=="Move")
       {
         ROS_INFO("PUBLSIH!!!!");
         //pub.publish(sequence.Mover());
         pub1.publish(sequence.Mover1());
         //ParkingGoalPublsiher();

       }
       else if(sequence.IsHaveOrder()=="LiftUp")
       {

       }
       else if(sequence.IsHaveOrder()=="LiftDown")
       {

       }

     }
     else
     {
       seq = sequence.GetSequence();
     }

     ros::spinOnce();
     rate.sleep();
   }
   return 0;
 };

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
    sequence.OrderParkOut(req.parkinglot,req.carNum);
  }

  //ParkingGoalPublsiher(PL[(int)req.parkinglot]);

  return true;
}

void ParkingGoalPublsiher(std::vector<double> goal)
{
  // geometry_msgs::PointStamped point;
  // point.header.frame_id = "map";

  // point.point.x = goal.at(0);
  // point.point.y = goal.at(1);
  // point.point.z = 0.0;

  // pub.publish(point);
  std::cout<<"qweqweqweqwe"<<std::endl;
  geometry_msgs::PoseStamped point1;
  point1.header.frame_id = "robot_1/map";
  point1.pose.position.x = goal.at(0);
  point1.pose.position.y = goal.at(1);
  

  point1.pose.orientation.x = 0;
  point1.pose.orientation.y = 0;
  point1.pose.orientation.z = 0;
  point1.pose.orientation.w = 1;

  pub1.publish(point1);
}
