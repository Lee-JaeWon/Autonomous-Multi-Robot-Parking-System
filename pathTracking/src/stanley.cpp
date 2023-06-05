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
#define STANLEY_K 1.0

class Stanley
{
public:

  std::vector<double> cyaw ;

private:
  //Parameters

  double k = 1.0;
  double k2 = 1.0;
  double v = 0.07;
  int hz = 33;

  //Time
  double nowTime;

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
  double error_front_axle;
  //


  int sign; //where is the robot (left or right side by path)

  nav_msgs::Path trajectory;
  int trajectory_length;
  geometry_msgs::Twist desired_robot_vel;

public:

  void Set_error_front_axle(double error_front_axle){
    this->error_front_axle = error_front_axle;
  }
  void Set_robot_pos(double x, double y, double theta, double time)
  {
    this->robot_pos_x = x;
    this->robot_pos_y = y;
    this->robot_heading = theta;
    this->nowTime = time;
  }

  void Set_robot_pos(double x, double y, double theta)
  {
    this->robot_pos_x = x;
    this->robot_pos_y = y;
    this->robot_heading = theta;
  }

  void Set_trajectory(nav_msgs::Path trajectory_)
  {
    this->trajectory=trajectory_;
    this->trajectory_length = (int)this->trajectory.poses.size();
    if(!this->cyaw.empty()) this->cyaw.clear();
  }


  void Set_parameters(double k_, double k2_, double v_, int hz_)
  {
    this->k = k_;
    this->k2 = k2_;
    this->v = v_;
    this->hz = hz_;
  }

  double return_Dist()
  {
    return this->distP2R;
  }

  void Calc_desired_point()
  {
    //calc desire pose, theta, dist
    double cur_px, cur_py, theta;
    double min_dist = std::numeric_limits<double>::infinity();
    int idx = 0;
    for(int i=0; i < this->trajectory_length ; i++){

      cur_px = this->trajectory.poses[i].pose.position.x;
      cur_py = this->trajectory.poses[i].pose.position.y;
      
      if ( i != this->trajectory_length -1 ){
      
      double dx = this->trajectory.poses[i+1].pose.position.x -this->trajectory.poses[i].pose.position.x;
      double dy = this->trajectory.poses[i+1].pose.position.y -this->trajectory.poses[i].pose.position.y;
      this->cyaw.push_back(atan2(dy,dx));
      
      }
      else this->cyaw.push_back(0.0);

      double dist_ = sqrt(( cur_px -this->robot_pos_x)*(cur_px-this->robot_pos_x) + (cur_py-this->robot_pos_y)*(cur_py-this->robot_pos_y));
      if(min_dist > dist_){
        min_dist  = dist_;
        this->desireNum = idx;
      }
      idx++;
    }
    double dx = this->robot_pos_x - this->trajectory.poses[this->desireNum].pose.position.x;
    double dy = this->robot_pos_y - this->trajectory.poses[this->desireNum].pose.position.y;

    double temp_error_front_axle = dx * -cos(this->robot_heading + M_PI_2) + dy * -sin(this->robot_heading + M_PI_2);
    this->Set_error_front_axle(temp_error_front_axle);

    this->distP2R = min_dist;
    //Delete an already passed trajectory
    for(int i=0;i<this->desireNum;i++){
      this->trajectory.poses.erase(this->trajectory.poses.begin());
      trajectory_length--;
    }


  }

  // End of Calc_desired_point

  double normalize_angle(double angle){
      while (angle > M_PI) angle -= 2.0 * M_PI;
      while (angle < -M_PI) angle += 2.0 * M_PI;
      return angle;
  }

  bool End_trajectory()
  {
    if(trajectory_length<3)
    {
      return true;
    }
    else return false;
  }

  geometry_msgs::Twist Get_vel()
  {
    this->Calc_desired_point();

    double theta_e = this->normalize_angle(this->cyaw[this->desireNum] - this->robot_heading);
    double linear_vel = 0.15;
    double theta_d = atan2(STANLEY_K * this->error_front_axle, linear_vel);
//    if(heading_error>2*PI) heading_error-=2*PI;
//    else if(heading_error<-2*PI) heading_error+=2*PI;

    // double steering_angle = this->k2 * (heading_error + sign * atan2(this->k * (this->distP2R), this->v));

    double angular_vel = theta_e * 1.5 + theta_d *1.25;//*this->hz;
    std::cout << "Path_angle : " << this->cyaw[this->desireNum] * (180.0 / M_PI)<< " Robot_yaw : " << this->robot_heading * (180.0 / M_PI)<< std::endl;
    std::cout << "theta_e : " << theta_e <<" theta_d : "<< theta_d << std::endl;

    //Saturation
    if(angular_vel>=0.6)
    {
      angular_vel = 0.6;
    }
    else if(angular_vel<=-0.6)
    {
      angular_vel = -0.6;
    }
    desired_robot_vel.linear.x = this->v - abs(angular_vel) * 0.3;
    if(theta_e > M_PI_2) desired_robot_vel.linear.x = 0;

    desired_robot_vel.angular.z = angular_vel;

    return desired_robot_vel;
  }

  geometry_msgs::Pose Get_Dp()
  {
    return this->trajectory.poses[0].pose;
  }

  nav_msgs::Path Left_Traj()
  {
    return this->trajectory;
  }

};
