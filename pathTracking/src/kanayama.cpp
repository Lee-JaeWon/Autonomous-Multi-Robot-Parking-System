#include <vector>
#include <cmath>
#include <limits>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Eigen>

#define PI 3.14

class Kanayama
{

private:
  //Parameters
  int hz = 33;
  double timeStep = 0.3; //0.3sec in a node
  double k_x, k_y, k_theta;
  int max_trajGap = 2;

  //Time
  double nowTime;
  int indexdjust=0;
  int closetIndex=0;

  //Now pose
  double robot_pos_x;
  double robot_pos_y;
  double robot_th;

  // Struct
  struct Pos {
    double x, y, theta;
  };

  //
  double delta_x, delta_y, delta_theta;
  double error_x, error_y, error_theta;
  int target_index;
  int last_index;

  double distP2R;

  int sign; //where is the robot (left or right side by path)

  nav_msgs::Path trajectory;
  nav_msgs::Path trajectory_copy;
  geometry_msgs::Twist trajectoryVel;
  int trajectory_length;
  geometry_msgs::Twist *q_r = (geometry_msgs::Twist *)malloc(sizeof(geometry_msgs::Twist)*100);
  Pos *p_r = (Pos *)malloc(sizeof(Pos)*100);
  geometry_msgs::Twist desired_robot_vel;

public:

  void Set_robot_pos(double x, double y, double theta, double time)
  {
    this->robot_pos_x = x;
    this->robot_pos_y = y;
    this->robot_th = theta;
    this->nowTime = time;
  }

  void Set_robot_pos(double x, double y, double theta)
  {
    this->robot_pos_x = x;
    this->robot_pos_y = y;
    this->robot_th = theta;
  }

  void Set_trajectory(nav_msgs::Path trajectory_)
  {
    //Init
    this->indexdjust=0;
    this->closetIndex=0;
    this->target_index=0;
    this->last_index=0;

    //path
    this->trajectory=trajectory_;
    this->trajectory_copy = this->trajectory;
    this->trajectory_length = (int)this->trajectory.poses.size();
    this->last_index = this->trajectory_length-1;

    //pos & vel
    q_r = (geometry_msgs::Twist *)realloc(q_r,sizeof(geometry_msgs::Twist) * this->trajectory_length);
    p_r = (Pos *)realloc(p_r,sizeof(Pos) * this->trajectory_length);

    for(int i=0;i<this->trajectory_length-1;i++)
    {
      double pre_x;
      double pre_y;
      double pre_th;

      if(i==0)
      {
        pre_x = this->robot_pos_x;
        pre_y = this->robot_pos_y;
      }
      else
      {
        pre_x = this->trajectory.poses[i-1].pose.position.x;
        pre_y = this->trajectory.poses[i-1].pose.position.y;
      }

      double now_x = this->trajectory.poses[i].pose.position.x;
      double now_y = this->trajectory.poses[i].pose.position.y;
      double after_x = this->trajectory.poses[i+1].pose.position.x;
      double after_y = this->trajectory.poses[i+1].pose.position.y;

      if(i==0)
      {
        pre_th = this->robot_th;
      }
      else
      {
        pre_th = atan2((now_y- pre_y),
                       (now_x - pre_x));
      }

      double now_th = atan2((after_y - now_y),
                            (after_x - now_x));

      p_r[i].x = now_x;
      p_r[i].y = now_y;
      p_r[i].theta = now_th;

      if(now_th>=0)
      {
        if(pre_th <= (now_th - PI))
        {
          pre_th+=2*PI;
        }
      }
      else if(now_th<0){
        if(pre_th >= (now_th + PI))
        {
          pre_th-=2*PI;
        }
      }

      q_r[i].angular.z = (now_th-pre_th)/this->timeStep;

      if(pre_th-now_th == 0)
      {
        q_r[i].linear.x = sqrt((after_x-now_x)*(after_x-now_x) + (after_y-now_y)*(after_y-now_y))/this->timeStep;
      }
      else {
        q_r[i].linear.x = sqrt((after_x-now_x)*(after_x-now_x) + (after_y-now_y)*(after_y-now_y)) * sqrt(2) / this->timeStep;
        //q_r[i].linear.x = sqrt((after_x-now_x)*(after_x-now_x) + (after_y-now_y)*(after_y-now_y))/2*q_r[i].angular.z/sin((now_th - pre_th)/2);
      }

//      std::cout <<"---------------------------"<<"\n";
//      std::cout <<"index        : "<<i<<"\n";
//      std::cout <<"now_th       : "<<now_th<<"\n";
//      std::cout <<"pre_th       : "<<pre_th<<"\n";

    }

    q_r[this->trajectory_length-1].linear.x  = 0.0;
    q_r[this->trajectory_length-1].angular.z = 0.0;

    p_r[this->trajectory_length-1].x = this->trajectory.poses[this->trajectory_length-1].pose.position.x;
    p_r[this->trajectory_length-1].y = this->trajectory.poses[this->trajectory_length-1].pose.position.x;
    p_r[this->trajectory_length-1].theta = p_r[this->trajectory_length-2].theta;
  }

  void Set_parameters(double k_x_, double k_y_, double timeStep_, int hz_)
  {
    this->k_x = k_x_;
    this->k_y = k_y_;
    this->k_theta = 2 * sqrt(this->k_y);
    this->timeStep = timeStep_;
    this->hz  = hz_;
  }

  double return_Dist()
  {
    return this->distP2R;
  }


  int Calc_Closest_Point()
  {
    //calc desire pose, theta, dist
    double x, y, theta;
    double min_dist = std::numeric_limits<double>::infinity();

    //Closest pose
    double c_pos_x;
    double c_pos_y;
    int cNum = 0;

    for(int i=0;i<this->trajectory_length;i++)
    {
      x = this->trajectory.poses[i].pose.position.x;
      y = this->trajectory.poses[i].pose.position.y;
      double dist_ = sqrt((x-this->robot_pos_x)*(x-this->robot_pos_x) + (y-this->robot_pos_y)*(y-this->robot_pos_y));

      if(min_dist > dist_)
      {
        c_pos_x = x;
        c_pos_y = y;
        double a11,a12,a21,a22;

        if(i==this->trajectory_length-1) {
          a11 = x-this->trajectory.poses[i-1].pose.position.x;
          a12 = y-this->trajectory.poses[i-1].pose.position.y;
        }
        else {
          a11 = this->trajectory.poses[i+1].pose.position.x-x;
          a12 = this->trajectory.poses[i+1].pose.position.y-y;
        }

        a21 = this->robot_pos_x-x;
        a22 = this->robot_pos_y-y;
        double Xprod = a11*a22-a12*a21;

        theta = atan2(a12,a11);

        if(Xprod >0){
            sign =-1;
        }
        else if(Xprod <0){
            sign=1;
        }
        else{
            sign =0;
        }

        min_dist = dist_;
        cNum = i;
      }
    }
    this->distP2R = min_dist;

    //Delete an already passed trajectory
    for(int i=0;i<cNum;i++)
    {
      this->trajectory.poses.erase(this->trajectory.poses.begin());
      trajectory_length--;
    }

    return cNum;
  }

  int calcIndex()
  {
    int index = floor(this->nowTime / this->timeStep);
    //if(index >= this->last_index) index = this->last_index;
    this->closetIndex += Calc_Closest_Point();
    if(index - this->closetIndex > this->max_trajGap)
    {
      this->indexdjust = -(index - this->closetIndex - this->max_trajGap);
    }

    this->target_index = index + this->indexdjust;

    if(this->target_index >= this->last_index) this->target_index = this->last_index;

    // std::cout <<"last_index      : "<<last_index<<"\n";
    // std::cout <<"index           : "<<index<<"\n";
    // std::cout <<"target_index    : "<<target_index<<"\n";
    // std::cout <<"closet_index    : "<<closetIndex<<"\n";
    // std::cout <<"indexdjust      : "<<this->indexdjust<<"\n";

    return this->target_index;
  }

  bool End_trajectory()
  {
    if(this->last_index == this->target_index && (this->target_index-this->closetIndex<3) )
    {
      return true;
    }
//    if(floor(this->nowTime / this->timeStep) > this->trajectory_length - 2)
//    {
//      return true;
//    }
    else return false;
  }

  geometry_msgs::Twist Get_vel()
  {
    //this->Calc_Closest_Point();
    //int index = floor(nowTime / 0.3);

    int index = this->calcIndex();

    //difference between theoretical pose (from the kinematic model) and estimated pose (from localization)
    delta_x = p_r[index].x - robot_pos_x;
    delta_y = p_r[index].y - robot_pos_y;

    if(p_r[index].theta >=0)
    {
      if(robot_th <= (p_r[index].theta  - PI))
      {
        robot_th+=2*PI;
      }
    }
    else if(p_r[index].theta <0){
      if(robot_th >= (p_r[index].theta  + PI))
      {
        robot_th-=2*PI;
      }
    }
    delta_theta = p_r[index].theta - robot_th;

    //error
    error_x = delta_x * cos(robot_th) + delta_y * sin(robot_th);
    error_y = -delta_x * sin(robot_th) + delta_y * cos(robot_th);
    error_theta = delta_theta;

    if(q_r[index].linear.x<0.1)
    {
      q_r[index].linear.x = 0.1;
    }

    double linear_vel = q_r[index].linear.x * cos(error_theta) + k_x * error_x;
    double angular_vel = q_r[index].angular.z + q_r[index].linear.x * (k_y * error_y + k_theta*sin(error_theta));

    std::cout <<"---------------------------"<<"\n";
//    std::cout <<"error_x      : "<<error_x<<"\n";
//    std::cout <<"error_y      : "<<error_y<<"\n";
//    std::cout <<"error_theta  : "<<error_theta<<"\n";
//    std::cout <<"cos(Etheta)  : "<<cos(error_theta)<<"\n";
//    std::cout <<"sin(Etheta)  : "<<sin(error_theta)<<"\n";
    std::cout <<"linear_vel   : "<<linear_vel<<"\n";
    std::cout <<"angular_vel  : "<<angular_vel<<"\n";
//    std::cout <<"q_r[index].angular.z      : "<<q_r[index].angular.z<<"\n";
//    std::cout <<"q_r[index].linear.x       : "<<q_r[index].linear.x<<"\n";
//    std::cout <<"k_y * error_y             : "<<k_y * error_y<<"\n";
//    std::cout <<"k_theta*sin(error_theta)  : "<<k_theta*sin(error_theta)<<"\n";
//    std::cout <<"k_x * error_x             : "<<k_x * error_x<<"\n";


    //Saturation
    if(angular_vel>=0.65)
    {
      angular_vel = 0.65;
    }
    else if(angular_vel<=-0.65)
    {
      angular_vel = -0.65;
    }

    if(linear_vel>=0.2)
    {
      linear_vel = 0.2;
    }
    else if(linear_vel<=-0.2)
    {
      linear_vel = -0.2;
    }

    desired_robot_vel.linear.x = linear_vel;
    desired_robot_vel.angular.z = angular_vel;
    return desired_robot_vel;
  }

  geometry_msgs::Pose Get_Dp()
  {
//    return this->trajectory.poses[floor(nowTime / 0.3)].pose;
//    return this->trajectory.poses[this->target_index].pose;
    return this->trajectory_copy.poses[this->target_index].pose;
  }

  nav_msgs::Path Left_Traj()
  {
    return this->trajectory;
  }

};
