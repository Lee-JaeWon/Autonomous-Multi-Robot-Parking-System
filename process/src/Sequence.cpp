#include <string>
#include <parking_msgs/action.h>
#include <parking_msgs/robotProcess.h>
#include <parking_msgs/miniSequence.h>
#include <parking_msgs/Sequence.h>
#include <geometry_msgs/PoseStamped.h>

class Sequence
{

private :
  int num=0;
  int robot_num=3;
  parking_msgs::Sequence Seq;
  std::vector<double> InputSpot;
  std::vector<double> OutputSpot;

  std::vector<double>* PL;
  
  std::vector<double> LiftingSpot;
  std::vector<double> LiftingSpot_two;

  int mini_seq_number=0;
  int robot_number=0;




  std::list<int> pocess_done_num_i;
  std::list<int> pocess_done_num_j;
  std::list<int> sequence_done_num_i;

public :
  std::list<int> robotNumList;
  std::list<geometry_msgs::PoseStamped> Pnt;
  std::list<std::string> orderType;
  std::list<std::vector<int>> locationList;

  std::list<std::vector<int>> *robot_stack;
  std::list<std::vector<int>> *robot_doing;

  // For parking strategy
  std::vector<bool> carList;
  std::vector<bool> paletteList;
  std::vector<int> robotList;

  //pub message
  geometry_msgs::PoseStamped order;

  //생성자
  Sequence() {

    Seq.header.frame_id="Multi-Robot-ParkingSystem";

  }

  //소멸자
  ~Sequence() {

  }

  void SetInOutSpot(std::vector<double> InputSpot, std::vector<double> OutputSpot)
  {
    this->InputSpot = InputSpot;
    this->OutputSpot = OutputSpot;
  }

  void SetRobotNum(int num)
  {
    this->robot_num = num;
    robot_stack = new std::list<std::vector<int>>[num];
    robot_doing = new std::list<std::vector<int>>[num];
  }

  void SetProcessDone(int i, int j, int k)
  {
    Seq.miniSequence[i].process[j].action[k].condition="Done";
  }

  void SetLists(std::vector<bool> cL, std::vector<bool> pL, std::vector<int> rL)
  {
    this->carList = cL;
    this->paletteList = pL;
    this->robotList = rL;
  }

  void SetPL(std::vector<double>* PL_)
  {
    PL = PL_;
  }

  void AddOrder(int type, int parkingNum, std::string carNum)
  {

  }

  void OrderParkIn(int parkingNum, std::vector<double> pos, std::string carNum)
  {
    parking_msgs::miniSequence mSeq;
    mSeq.condition = "NotStart";
    mSeq.order="ParkIn";
    mSeq.SequenceNumber=mini_seq_number++;




    //실제 주차하는 로봇
    if(robotList.at(robotList.size()-2)!=0)
    {
      parking_msgs::robotProcess rP;
      int action_number=0;

      //LiftUp
      parking_msgs::action act0;
      act0.number = action_number++;
      act0.condition = "NotStart";
      act0.action = "LiftUp";
      act0.parkingLot = 999;

      //Move to parking lot
      parking_msgs::action act1;
      act1.number = action_number++;
      act1.condition = "NotStart";
      act1.action = "Move";
      act1.parkingLot = parkingNum;
      act1.x = pos.at(0);
      act1.y = pos.at(1);
      act1.orientation = 0.0;

      //LiftDown
      parking_msgs::action act2;
      act2.number = action_number++;
      act2.condition = "NotStart";
      act2.action = "LiftDown";
      act2.parkingLot = 999;


      rP.job="Parker";
      rP.condition = "NotStart";
      //현재 InputLot에 있는 로봇
      rP.robotNumber=(robotList.at(robotList.size()-2)-1);

      rP.action.push_back(act0);
      rP.action.push_back(act1);
      rP.action.push_back(act2);

      mSeq.process.push_back(rP);

      //List 재구성
      carList.at(parkingNum)=true;

      std::swap(robotList[parkingNum],robotList[robotList.size()-2]);

      std::swap(paletteList[parkingNum],paletteList[paletteList.size()-2]);

    }
    //주차장을 다시 채우는 로봇
    for(int i=0;i<carList.size()-2;i++)
    {
      if(carList.at(i)==false && robotList.at(i)!=0 && paletteList.at(i)==true)
      {
        parking_msgs::robotProcess rP;
        int action_number=0;

        //LiftUp
        parking_msgs::action act0;
        act0.number = action_number++;
        act0.condition = "NotStart";
        act0.action = "LiftUp";
        act0.parkingLot = 999;

        //Move to parking lot
        parking_msgs::action act1;
        act1.number = action_number++;
        act1.condition = "NotStart";
        act1.action = "Move";
        act1.parkingLot = 999;
        act1.x = InputSpot.at(0);
        act1.y = InputSpot.at(1);
        act1.orientation = 0.0;

        //LiftDown
        parking_msgs::action act2;
        act2.number = action_number++;
        act2.condition = "NotStart";
        act2.action = "LiftDown";
        act2.parkingLot = 999;


        rP.job="Mover";
        rP.condition = "NotStart";

        //현재 차는 없고 팔레트는 있는 로봇
        rP.robotNumber=(robotList.at(i)-1);

        rP.action.push_back(act0);
        rP.action.push_back(act1);
        rP.action.push_back(act2);

        mSeq.process.push_back(rP);

        //List 재구성

        std::swap(robotList[i],robotList[robotList.size()-2]);

        std::swap(paletteList[i],paletteList[paletteList.size()-2]);


        break;
      }
    }

    //차량을 옮긴차는 다시 차량없는 대기장소로 이동
    for(int i=0; i<carList.size()-2;i++)
    {
      //로봇도 없고 차도 없고 팔레트만 있는 장소 중 제일 앞번호?
      if(carList.at(i)==false && robotList.at(i)==0 && paletteList.at(i)==true)
      {
        parking_msgs::robotProcess rP;
        int action_number=0;

        //Move to Empty lot for Ready
        parking_msgs::action act0;
        act0.number = action_number++;
        act0.condition = "NotStart";
        act0.action = "Move";
        act0.parkingLot = 999;
        act0.x = PL[i].at(0);
        act0.y = PL[i].at(1);
        act0.orientation = 0.0;

        rP.job="Mover";
        rP.condition = "NotStart";

        rP.robotNumber = (robotList.at(parkingNum)-1);

        rP.action.push_back(act0);

        mSeq.process.push_back(rP);

        //List 재구성

        std::swap(robotList[i],robotList[parkingNum]);

        break;
      }
    }

    Seq.miniSequence.push_back(mSeq);
  }

  void OrderParkOut(int parkingNum, std::vector<double> pos, std::string carNum)
  {
    parking_msgs::miniSequence mSeq;
    mSeq.condition = "NotStart";
    mSeq.order="ParkOut";
    mSeq.SequenceNumber=mini_seq_number++;

    for(int i=carList.size()-3; i>=0; i--)
    {
      if(paletteList.at(i)==true && robotList.at(i)!=0)
      {
        parking_msgs::robotProcess rP;
        int action_number=0;

        //Move to Target Car
        parking_msgs::action act0;
        act0.number = action_number++;
        act0.condition = "NotStart";
        act0.action = "Move";
        act0.parkingLot = parkingNum;
        act0.x = pos.at(0);
        act0.y = pos.at(1);
        act0.orientation = 0.0;

        //LiftUp
        parking_msgs::action act1;
        act1.number = action_number++;
        act1.condition = "NotStart";
        act1.action = "LiftUp";
        act1.parkingLot = 999;

        //Move to OutputLot
        parking_msgs::action act2;
        act2.number = action_number++;
        act2.condition = "NotStart";
        act2.action = "Move";
        act2.parkingLot = parkingNum;
        act2.x = OutputSpot.at(0);
        act2.y = OutputSpot.at(1);
        act2.orientation = 0.0;

        //LiftDown
        parking_msgs::action act3;
        act3.number = action_number++;
        act3.condition = "NotStart";
        act3.action = "LiftDown";
        act3.parkingLot = 999;

        //Wait
        //딜레이 몇초 넣어야 할듯?

        //LiftUp
        parking_msgs::action act4;
        act4.number = action_number++;
        act4.condition = "NotStart";
        act4.action = "LiftUp";
        act4.parkingLot = 999;

        //Move back
        parking_msgs::action act5;
        act5.number = action_number++;
        act5.condition = "NotStart";
        act5.action = "Move";
        act5.parkingLot = parkingNum;
        act5.x = pos.at(0);
        act5.y = pos.at(1);
        act5.orientation = 0.0;

        //LiftDown
        parking_msgs::action act6;
        act6.number = action_number++;
        act6.condition = "NotStart";
        act6.action = "LiftDown";
        act6.parkingLot = 999;

        rP.job="ParkOuter";
        rP.condition = "NotStart";

        rP.robotNumber = (robotList.at(robotList.size()-2)-1);

        rP.action.push_back(act0);

        mSeq.process.push_back(rP);

        //List 재구성
        carList.at(parkingNum)=false;

        std::swap(robotList[i],robotList[parkingNum]);

        break;
      }
    }
    Seq.miniSequence.push_back(mSeq);
  }

  void RemoveProcess()
  {
    if(!pocess_done_num_i.empty())
    {
      int i= pocess_done_num_i.front();
      pocess_done_num_i.pop_front();
      int j= pocess_done_num_j.front();
      pocess_done_num_j.pop_front();
      Seq.miniSequence[i].process.erase(Seq.miniSequence[i].process.begin() + j);
    }
  }

  void RemoveSequence()
  {
    if(!sequence_done_num_i.empty())
    {
      int i= sequence_done_num_i.front();
      sequence_done_num_i.pop_front();
      Seq.miniSequence.erase(Seq.miniSequence.begin() + i);
    }
  }

  void Seq2Stack()
  {
    // 스택 갱신
    std::list<std::vector<int>> *stack = new std::list<std::vector<int>>[robot_num];
    for(int i=0; i<Seq.miniSequence.size();i++)
    {
      if (Seq.miniSequence[i].condition=="Done") continue;

      bool sequenceDone = true;

      for(int j=0; j<Seq.miniSequence[i].process.size();j++)
      {
        if (Seq.miniSequence[i].process[j].condition=="Done") continue;

        //모든 명령이 Done 했는지 확인 위한 변수
        bool processDone = true;

        for(int k=0; k<Seq.miniSequence[i].process[j].action.size();k++)
        {
          if(Seq.miniSequence[i].process[j].action[k].condition=="NotStart")
          {
            std::vector<int> v;
            v.push_back(i);
            v.push_back(j);
            v.push_back(k);

            int rNum = Seq.miniSequence[i].process[j].robotNumber;
            stack[rNum].push_back(v);
            robot_stack[rNum] = stack[rNum];
          }

          if(Seq.miniSequence[i].process[j].action[k].condition!="Done")
            processDone = false;
        }

        if(processDone)
          Seq.miniSequence[i].process[j].condition = "Done";

        if(Seq.miniSequence[i].process[j].condition!="Done")
          sequenceDone = false;
      }

      if(sequenceDone)
        Seq.miniSequence[i].condition = "Done";
    }
  }

  void PopStack()
  {
    for(int i=0;i<robot_num;i++)
    {
      if(robot_doing[i].empty() && !robot_stack[i].empty())
      {
        robot_doing[i].push_back(robot_stack[i].front());
        robot_stack[i].pop_front();
      }
    }
  }

  std::vector<int> GiveOrder(int n)
  {
    if(!robot_doing[n].empty())
    {
      int i = robot_doing[n].front().at(0);
      int j = robot_doing[n].front().at(1);
      int k = robot_doing[n].front().at(2);
      Seq.miniSequence[i].condition="Working";
      Seq.miniSequence[i].process[j].condition="Working";
      Seq.miniSequence[i].process[j].action[k].condition="Working";

      return robot_doing[n].front();
    }
  }

  bool CheckOrderEmpty(int n)
  {
    if(robot_doing[n].size()==1)
    {
      robot_doing[n].push_back(robot_doing[n].front());
      return true;
    }
    else
    {
      return false;
    }
  }

  void SetProcessDone(int i, int j, int k, int rNum)
  {
    if(robot_doing[rNum].front().at(0)==i &&
              robot_doing[rNum].front().at(1)==j &&
                    robot_doing[rNum].front().at(2)==k)
    {
      Seq.miniSequence[i].process[j].action[k].condition="Done";
      robot_doing[rNum].clear();
    }
  }

  //Getter
  parking_msgs::Sequence GetSequence()
  {
    return Seq;
  }

};
