#include <string>
#include <parking_msgs/action.h>
#include <parking_msgs/robotProcess.h>
#include <parking_msgs/miniSequence.h>
#include <parking_msgs/Sequence.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class Sequence
{

private :
  int a;
  parking_msgs::Sequence Seq;

public :
  std::list<geometry_msgs::PointStamped> Pnt;
  std::list<std::string> orderType;

  //pub message
  geometry_msgs::PointStamped order;

  //생성자
  Sequence() {

    Seq.header.frame_id="Multi-Robot-ParkingSystem";

  }

  //소멸자
  ~Sequence() {

  }

  void AddOrder(int type, int parkingNum, std::string carNum)
  {

  }

  void OrderParkIn(int parkingNum, std::vector<double> pos, std::string carNum)
  {
    parking_msgs::action act;
    act.condition = "NotStart";
    act.action = "Move";
    act.x = pos.at(0);
    act.y = pos.at(1);
    act.orientation = 0.0;

    parking_msgs::robotProcess rP;
    rP.job="Parker";
    rP.robotNumber=0;
    rP.action.push_back(act);

    parking_msgs::miniSequence mSeq;
    mSeq.condition = "NotStart";
    mSeq.order="ParkIn";
    mSeq.process.push_back(rP);

    Seq.miniSequence.push_back(mSeq);
  }

  void OrderParkOut(int parkingNum, std::string carNum)
  {

  }

  void CheckSequence()
  {
    Seq.miniSequence.size();
    for(int i=0; i<(int)Seq.miniSequence.size(); i++)
    {
      if(Seq.miniSequence[i].condition!="Done")
      {
        Seq.miniSequence[i].condition="Working";

        for(int j=0; j<(int)Seq.miniSequence[i].process.size(); j++)
        {
          for(int k=0; k<(int)Seq.miniSequence[i].process[j].action.size(); k++)
          {
            if(Seq.miniSequence[i].process[j].action[k].condition=="NotStart")
            {
              if(Seq.miniSequence[i].process[j].action[k].action=="Move")
              {
                geometry_msgs::PointStamped ps;
                ps.header.frame_id = "robot_1/map";
                ps.point.x = Seq.miniSequence[i].process[j].action[k].x;
                ps.point.y = Seq.miniSequence[i].process[j].action[k].y;
                Pnt.push_back(ps);
                orderType.push_back(Seq.miniSequence[i].process[j].action[k].action);
              }
              else if(Seq.miniSequence[i].process[j].action[k].action=="LiftUp")
              {

              }
              else if(Seq.miniSequence[i].process[j].action[k].action=="LiftDown")
              {

              }
              Seq.miniSequence[i].process[j].action[k].condition="Working";

            }
          }
        }
      }
    }
  }

  std::string IsHaveOrder()
  {
    CheckSequence();

    if(orderType.size()>0)
    {
      if(orderType.front()=="Move")
      {
        return "Move";
      }
      else if(orderType.front()=="LiftUp")
      {
        return "LiftUp";
      }
      else if(orderType.front()=="LiftDown")
      {
        return "LiftDown";
      }
      else
      {
        return "None";
      }
    }
    else {
      return "None";
    }
  }

  geometry_msgs::PointStamped Mover()
  {
    geometry_msgs::PointStamped ps;
    if(orderType.front()=="Move")
    {
      ps = Pnt.front();
      Pnt.pop_front();
      orderType.pop_front();
    }
    return ps;
  }

  geometry_msgs::PoseStamped Mover1()
  {
    geometry_msgs::PoseStamped ps;
    if(orderType.front()=="Move")
    {
      ps.header.frame_id="robot_1/map";
      ps.pose.position.x=Pnt.front().point.x;
      ps.pose.position.y=Pnt.front().point.y;
      ps.pose.position.z=Pnt.front().point.z;
      Pnt.pop_front();
      orderType.pop_front();
    }
    return ps;
  }

  void Lifter()
  {
    if(orderType.front()=="LiftUp")
    {
      geometry_msgs::PointStamped ps = Pnt.front();
      Pnt.pop_front();
      orderType.pop_front();
    }
    else if(orderType.front()=="LiftDown")
    {
      orderType.pop_front();
    }
  }

  //Getter
  parking_msgs::Sequence GetSequence()
  {
    return Seq;
  }

};
