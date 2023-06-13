#include <string>
#include <parking_msgs/action.h>
#include <parking_msgs/robotProcess.h>
#include <parking_msgs/miniSequence.h>
#include <parking_msgs/Sequence.h>
#include <geometry_msgs/PoseStamped.h>

class Sequence
{

private:
  int num = 0;
  int robot_num = 3;
  parking_msgs::Sequence Seq;
  std::vector<double> InputSpot;
  std::vector<double> OutputSpot;

  std::vector<double> LiftingSpot;
  std::vector<double> LiftingSpot_two;


  int mini_seq_number = 0;
  int robot_number = 0;
  int action_number = 0;

  std::list<int> pocess_done_num_i;
  std::list<int> pocess_done_num_j;
  std::list<int> sequence_done_num_i;

public:
  std::list<int> robotNumList;
  std::list<geometry_msgs::PoseStamped> Pnt;
  std::list<std::string> orderType;
  std::list<std::vector<int>> locationList;

  // pub message
  geometry_msgs::PoseStamped order;

  // 생성자
  Sequence()
  {

    Seq.header.frame_id = "Multi-Robot-ParkingSystem";
  }

  // 소멸자
  ~Sequence()
  {
  }

  void SetInOutSpot(std::vector<double> InputSpot, std::vector<double> OutputSpot)
  {
    this->InputSpot = InputSpot;
    this->OutputSpot = OutputSpot;
  }

  void SetLiftingSpot(std::vector<double> LiftingSpot, std::vector<double> LiftingSpot_two)
  {
    this->LiftingSpot = LiftingSpot;
    this->LiftingSpot_two = LiftingSpot_two;
  }

  void SetRobotNum(int num)
  {
    this->robot_num = num;
  }

  void SetProcessDone(int i, int j, int k)
  {
    Seq.miniSequence[i].process[j].action[k].condition = "Done";
  }

  void AddOrder(int type, int parkingNum, std::string carNum)
  {
  }

  void OrderParkIn(int parkingNum, std::vector<double> pos, std::string carNum)
  {

    // Move to parking lot1
    parking_msgs::action act0;
    act0.number = action_number++;
    act0.condition = "NotStart";
    act0.action = "Move";
    act0.parkingLot = parkingNum;
    act0.x = pos.at(0);
    act0.y = pos.at(1);
    act0.orientation = 0.0;

    // Move to parking lot2
    parking_msgs::action act1;
    act1.number = action_number++;
    act1.condition = "NotStart";
    act1.action = "Move";
    act1.parkingLot = 999;
    act1.x = LiftingSpot.at(0);
    act1.y = LiftingSpot.at(1);
    act1.orientation = 0.0;

    parking_msgs::robotProcess rP;
    rP.job = "Parker";
    rP.condition = "NotStart";
    rP.robotNumber = ((num++) % robot_num);
    rP.action.push_back(act0);
    rP.action.push_back(act1);

    // Parkinglot에 로봇 다시 채우기
    action_number = 0;
    parking_msgs::action act2;
    act2.number = action_number++;
    act2.condition = "NotStart";
    act2.action = "Move";
    act2.parkingLot = 1000;
    act2.x = LiftingSpot_two.at(0);
    act2.y = LiftingSpot_two.at(1);
    act2.orientation = 0.0;

    parking_msgs::action act3;
    act3.number = action_number++;
    act3.condition = "NotStart";
    act3.action = "Move";
    act3.parkingLot = 1000;
    act3.x = InputSpot.at(0);
    act3.y = InputSpot.at(1);
    act3.orientation = 0.0;

    parking_msgs::robotProcess rP2;
    rP2.job = "Mover_IL";
    rP2.condition = "NotStart";
    rP2.robotNumber = ((num++) % robot_num);

    rP2.action.push_back(act2);
    rP2.action.push_back(act3);

    // Parkinglot에 로봇 다시 채우기
    parking_msgs::action act4;
    act4.number = action_number++;
    act4.condition = "NotStart";
    act4.action = "Move";
    act4.parkingLot = 1000;
    act4.x = OutputSpot.at(0);
    act4.y = OutputSpot.at(1);
    act4.orientation = 0.0;

    parking_msgs::action act5;
    act5.number = action_number++;
    act5.condition = "NotStart";
    act5.action = "Move";
    act5.parkingLot = 1000;
    act5.x = pos.at(0);
    act5.y = pos.at(1);
    act5.orientation = 0.0;

    parking_msgs::robotProcess rP3;
    rP3.job = "Mover";
    rP3.robotNumber = ((num++) % robot_num);;
    rP3.action.push_back(act4);
    rP3.action.push_back(act5);

    // 각 로봇 process 채워넣기
    parking_msgs::miniSequence mSeq;
    mSeq.condition = "NotStart";
    mSeq.order = "ParkIn";
    mSeq.SequenceNumber = mini_seq_number++;
    mSeq.process.push_back(rP);
    mSeq.process.push_back(rP2);
    mSeq.process.push_back(rP3);

    Seq.miniSequence.push_back(mSeq);
  }

  void OrderParkOut(int parkingNum, std::string carNum)
  {
  }

  void RemoveProcess()
  {
    if (!pocess_done_num_i.empty())
    {
      int i = pocess_done_num_i.front();
      pocess_done_num_i.pop_front();
      int j = pocess_done_num_j.front();
      pocess_done_num_j.pop_front();
      Seq.miniSequence[i].process.erase(Seq.miniSequence[i].process.begin() + j);
    }
  }

  void RemoveSequence()
  {
    if (!sequence_done_num_i.empty())
    {
      int i = sequence_done_num_i.front();
      sequence_done_num_i.pop_front();
      Seq.miniSequence.erase(Seq.miniSequence.begin() + i);
    }
  }

  void CheckSequence()
  {
    // RemoveProcess();
    int sequence_size = (int)Seq.miniSequence.size();

    bool sequenceDone = true;
    for (int i = 0; i < sequence_size; i++) // 모든 miniSequence 탐색
    {
      if (Seq.miniSequence[i].condition != "Done")
      {
        Seq.miniSequence[i].condition = "Working";

        int process_size = (int)Seq.miniSequence[i].process.size();
        for (int j = 0; j < process_size; j++) // 하나의 miniSequence에 필요한 모든 로봇의 개수만큼 탐색
        {
          if (Seq.miniSequence[i].process[j].condition != "Done")
          {
            sequenceDone = false;
          }
          // 모든 명령이 Done 했는지 확인은 위한 변수
          bool processDone = true;

          int action_size = (int)Seq.miniSequence[i].process[j].action.size();
          for (int k = 0; k < action_size; k++)
          {
            // 첫번째 명령 + 아직시작안했는지 OR 첫번째 명령이 아니라면 + 이전명령이 끝난상태인지 + 현재명령이 아직 시작 안했는지
            if ((k == 0 && Seq.miniSequence[i].process[j].action[k].condition == "NotStart") || (k != 0 && Seq.miniSequence[i].process[j].action[k - 1].condition == "Done" && Seq.miniSequence[i].process[j].action[k].condition == "NotStart"))
            {
              processDone = false;

              // Move 명령이라면
              if (Seq.miniSequence[i].process[j].action[k].action == "Move")
              {
                int robot_num = Seq.miniSequence[i].process[j].robotNumber;
                geometry_msgs::PoseStamped ps;
                ps.header.frame_id = "map";
                ps.pose.position.x = Seq.miniSequence[i].process[j].action[k].x;
                ps.pose.position.y = Seq.miniSequence[i].process[j].action[k].y;
                Pnt.push_back(ps);
                robotNumList.push_back(robot_num);

                // put the location of process
                std::vector<int> v;
                v.push_back(i);
                v.push_back(j);
                v.push_back(k);
                locationList.push_back(v);

                orderType.push_back(Seq.miniSequence[i].process[j].action[k].action);
              }
              // Lift Up 명령이라면
              else if (Seq.miniSequence[i].process[j].action[k].action == "LiftUp")
              {
              }
              else if (Seq.miniSequence[i].process[j].action[k].action == "LiftDown")
              {
              }
              Seq.miniSequence[i].process[j].condition = "Working";
              Seq.miniSequence[i].process[j].action[k].condition = "Working";
              break;
            }
            else if (Seq.miniSequence[i].process[j].action[k].condition == "Done")
            {
              // processDone = true;
              // std::cout<<"done - "<<j<<std::endl;
            }
            else
            {
              processDone = false;
            }
          }

          // 프로세스 내부가 다 Done이면
          if (processDone)
          {
            // std::cout<<"process donne - "<<j<<std::endl;
            Seq.miniSequence[i].process[j].condition = "Done";
            // pocess_done_num_i.push_back(i);
            // pocess_done_num_j.push_back(j);
          }
        }
      }

      // 시퀀스 내부가 다 Done이면

      if (sequenceDone)
      {
        Seq.miniSequence[i].condition = "Done";
        sequence_done_num_i.push_back(i);
      }
    }
  }

  std::string IsHaveOrder()
  {
    CheckSequence();

    if (orderType.size() > 0)
    {
      return orderType.front();
    }
    else
    {
      return "None";
    }
  }

  geometry_msgs::PoseStamped Mover()
  {
    geometry_msgs::PoseStamped ps;

    if (orderType.size() > 0 && orderType.front() == "Move")
    {
      ps = Pnt.front();
      Pnt.pop_front();
      orderType.pop_front();
    }
    return ps;
  }

  int getRobotNum()
  {
    int num;
    num = robotNumList.front();
    robotNumList.pop_front();
    return num;
  }

  std::vector<int> getLocation()
  {
    std::vector<int> v;
    v = locationList.front();
    locationList.pop_front();
    return v;
  }

  void Lifter()
  {
    if (orderType.size() > 0 && orderType.front() == "LiftUp")
    {
      geometry_msgs::PoseStamped ps = Pnt.front();
      Pnt.pop_front();
      orderType.pop_front();
    }
    else if (orderType.front() == "LiftDown")
    {
      orderType.pop_front();
    }
  }

  // Getter
  parking_msgs::Sequence GetSequence()
  {
    return Seq;
  }
};
