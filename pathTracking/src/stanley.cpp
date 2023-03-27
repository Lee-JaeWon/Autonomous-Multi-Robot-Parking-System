#include <vector>
#include <cmath>
#include <limits>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14

class Stanley
{
public:

  //vector<double> trajectory[1000000];
  //void run();

private:
  //Parameters
  double k = 1.0;
  double k2 = 1.0;
  double v = 1.0;
  int hz = 33;

  //Now pose
  double robot_pos_x;
  double robot_pos_y;
  double robot_heading;

  //Desire(?) pose
  int desireNum;
  double desire_x;
  double desire_y;
  double desire_heading;
  double distP2R;

  int sign; //where is the robot (left or right side by path)

  nav_msgs::Path trajectory;
  int trajectory_length;
  geometry_msgs::Twist desired_robot_vel;

public:

  void Set_robot_pos(double x, double y, double theta)
  {
    this->robot_pos_x = x;
    this->robot_pos_y = y;
    this->robot_heading = theta;

//    if(theta>PI) this->robot_heading = theta - (2*PI);
//    else if(theta<-PI) this->robot_heading = theta + (2*PI);
//    else this->robot_heading = theta;
  }

  void Set_trajectory(nav_msgs::Path trajectory_)
  {
    this->trajectory=trajectory_;
    this->trajectory_length = this->trajectory.poses.size();
  }

  void Set_parameters(double k_, double k2_, double v_, int hz_)
  {
    this->k = k_;
    this->k2 = k2_;
    this->v = v_;
    this->hz = hz_;
  }

  void Calc_desired_point()
  {
    //calc desire pose, theta, dist
    double x, y, theta;
    double min_dist = std::numeric_limits<double>::infinity();
    for(int i=0;i<this->trajectory_length;i++)
    {
      x = this->trajectory.poses[i].pose.position.x;
      y = this->trajectory.poses[i].pose.position.y;
      double dist_ = (x-this->robot_pos_x)*(x-this->robot_pos_x) + (y-this->robot_pos_y)*(y-this->robot_pos_y);

      if(min_dist > dist_)
      {
        this->desire_x = x;
        this->desire_y = y;
        double a11,a12,a21,a22;

        if(i==this->trajectory_length-1) {
          //theta = atan2(y-this->trajectory.poses[i-1].pose.position.y, x-this->trajectory.poses[i-1].pose.position.x);
          a11 = x-this->trajectory.poses[i-1].pose.position.x;
          a12 = y-this->trajectory.poses[i-1].pose.position.y;
        }
        else {
          //theta = atan2(this->trajectory.poses[i+1].pose.position.y-y, this->trajectory.poses[i+1].pose.position.x-x);
          a11 = this->trajectory.poses[i+1].pose.position.x-x;
          a12 = this->trajectory.poses[i+1].pose.position.y-y;
        }

        a21 = this->robot_pos_x-x;
        a22 = this->robot_pos_y-y;
        double Xprod = a11*a22-a12*a21;

        theta = atan2(a12,a11);

//        if(theta>PI) theta -=(2*PI);
//        else if(theta<-PI) theta +=(2*PI);

        if(Xprod >0){
            sign =-1;
            //std::cout <<"Aside"<<"\n"; //LEFT
        }
        else if(Xprod <0){
            sign=1;
            //std::cout <<"Bside"<<"\n"; //RIGHT
        }
        else{
            sign =0;
        }


        min_dist = dist_;
        this->desireNum = i;
        this->desire_heading = theta;
      }
    }
    this->distP2R = min_dist;

    //Delete an already passed trajectory
    for(int i=0;i<this->desireNum;i++)
    {
      this->trajectory.poses.erase(this->trajectory.poses.begin());
      trajectory_length--;
    }
    //std::cout<<trajectory_length<<"\n";
  }

  bool End_trajectory()
  {
    if(trajectory_length<5)
    {
      return true;
    }
    else return false;
  }

  geometry_msgs::Twist Get_vel()
  {
    this->Calc_desired_point();

//    if(this->desire_heading>0)
//    {
//      if(this->robot_heading<0)
//      {
//        this->robot_heading+=2*PI;
//      }
//    }
//    else {
//      if(this->robot_heading>0)
//      {
//        this->robot_heading-=2*PI;
//      }
//    }

    if(this->desire_heading>=0)
    {
      if(this->robot_heading <= (this->desire_heading - PI))
      {
        this->robot_heading+=2*PI;
      }
    }
    else if(this->desire_heading<0){
      if(this->robot_heading >= (this->desire_heading + PI))
      {
        this->robot_heading-=2*PI;
      }
    }




    double heading_error = this->desire_heading-this->robot_heading;

//    if(heading_error>2*PI) heading_error-=2*PI;
//    else if(heading_error<-2*PI) heading_error+=2*PI;

    double steering_angle = this->k2 * (heading_error + sign * atan2(this->k * (this->distP2R), this->v));
    double angular_vel = steering_angle;//*this->hz;

//    std::cout <<"theta(path angle) : "<<this->desire_heading<<"\n";
//    std::cout <<"robot head angle  : "<<this->robot_heading<<"\n";
//    std::cout <<"heading_error  : "<<heading_error<<"\n";
//    std::cout <<"angular_vel  : "<<angular_vel<<"\n";

    //Saturation
    if(angular_vel>=3)
    {
      angular_vel = 3;
    }
    else if(angular_vel<=-3)
    {
      angular_vel = -3;
    }

    desired_robot_vel.linear.x = this->v;
    desired_robot_vel.angular.z = angular_vel;

    return desired_robot_vel;
  }

  geometry_msgs::Pose Get_Dp()
  {
    return this->trajectory.poses[0].pose;
  }

};
