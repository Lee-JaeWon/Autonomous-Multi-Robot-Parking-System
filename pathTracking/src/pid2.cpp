#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#define PI 3.14
class pid2
{

private:
    // parameter for tracking control
    double p_h = 0.0;
    double i_h = 0.0;
    double d_h = 0.0;
    double p_c = 0.0;
    double i_c = 0.0;
    double d_c = 0.0;
    double v = 0.0;
    double hz = 0.0;
    double dt = 1 / hz;
    double heading_err = 0.0;
    double pre_heading_err = 0.0;
    double heading_err_sum = 0.0;
    double dist_err = 0.0;
    double pre_dist_err = 0.0;
    double dist = 0.0;
    double dist_min = 100000000.0;
    double desire_x = 0.0;
    double desire_y = 0.0;
    double desire_theta = 0.0; // rad
    double dist_err_sum = 0.0;
    double steering = 0.0;
    double steering_head = 0.0;
    double sterring_ctr = 0.0;
    double calsteer_x = 0.0;
    double calsteer_y = 0.0;
    double cnt = 0;

    int sign = 0;

    // robot condition
    double robot_pos_x = 0.0;     // m
    double robot_pos_y = 0.0;     // m
    double robot_ori_theta = 0.0; // rad

    int trajectory_length;
    int index = 0;

    bool arrive_goal = false;
    nav_msgs::Path trajectory;
    geometry_msgs::PoseStamped goal;
    geometry_msgs::Twist desired_robot_vel;

public:
    void Set_robot_pos(double x, double y, double theta) // set robot pos
    {
        this->robot_pos_x = x;
        this->robot_pos_y = y;
        this->robot_ori_theta = theta;
    }

    void Set_trajectory(nav_msgs::Path trajectory_) // set path
    {
        this->trajectory = trajectory_;
        this->trajectory_length = this->trajectory.poses.size();
    }
    void Set_goal(geometry_msgs::PoseStamped goal_) // set goal point
    {
        this->goal = goal_;
    }
    void Set_parameters(double p_h_, double i_h_, double d_h, double p_c_, double i_c_, double d_c_, double hz_, double v_)
    {
        this->p_h = p_h_;
        this->i_h = i_h_;
        this->d_h = d_h;
        this->p_c = p_c_;
        this->i_c = i_c_;
        this->d_c = d_c_;
        this->hz = hz_;
        this->v = v_;
    }
    bool End_trajectory()
    {
        if (this->trajectory_length < 3)
        {
            return true;
        }
        else
            return false;
    }
    geometry_msgs::Twist Get_vel()
    {
        this->cnt = cnt + 1;
        // this->dist_min = 10000000000.0;
        index = 0;
        for (int i = 0; i < this->trajectory_length; i++)
        {
            this->desire_x = this->trajectory.poses[i].pose.position.x;
            this->desire_y = this->trajectory.poses[i].pose.position.y;
            dist = sqrt((this->robot_pos_x - this->desire_x) * (this->robot_pos_x - this->desire_x) + (this->robot_pos_y - this->desire_y) * (this->robot_pos_y - this->desire_y));
            if (dist < this->dist_min)
            {
                this->dist_min = dist;

                index = i;
            }
        }

        // ROS_INFO("index = %d",index);
        index += 30;

        this->desire_x = this->trajectory.poses[index].pose.position.x;
        this->desire_y = this->trajectory.poses[index].pose.position.y;
        this->calsteer_x = this->robot_pos_x - this->desire_x;
        this->calsteer_y = this->robot_pos_y - this->desire_y;
        if (this->trajectory.poses[index + 1].pose.position.x - this->trajectory.poses[index].pose.position.x >= 0)
        {
            if (this->calsteer_y >= 0)
                this->sign = -1;
            else
                this->sign = 1;
        }

        else if (this->trajectory.poses[index + 1].pose.position.x - this->trajectory.poses[index].pose.position.x < 0)
        {
            if (this->calsteer_y >= 0)
                this->sign = 1;
            else
                this->sign = -1;
        }
        else if (this->trajectory.poses[index + 1].pose.position.y - this->trajectory.poses[index].pose.position.y <= 0)
        {
            if (this->calsteer_x >= 0)
                this->sign = -1;
            else
                this->sign = 1;
        }
        else if (this->trajectory.poses[index + 1].pose.position.y - this->trajectory.poses[index].pose.position.y < 0)
        {
            if (this->calsteer_x >= 0)
                this->sign = 1;
            else
                this->sign = -1;
        }

        //ROS_INFO("index = %d", this->index);
        double temp1;
        double temp2;
        if (index == this->trajectory_length - 1)
        {
            
            // ROS_INFO("index = %d",index);
            ROS_INFO("trajectory_length = %d",this->trajectory_length-1);
            arrive_goal = true;
        }
        else
        {
            temp1 = this->trajectory.poses[index + 1].pose.position.x - this->trajectory.poses[index].pose.position.x;
            temp2 = this->trajectory.poses[index + 1].pose.position.y - this->trajectory.poses[index].pose.position.y;
            arrive_goal = false;
        }

        this->desire_theta = atan2(temp2, temp1);

        if (this->desire_theta >= 0)
        {
            if (this->robot_ori_theta <= (this->desire_theta - PI))
            {
                this->robot_ori_theta += 2 * PI;
            }
        }
        else if (this->desire_theta < 0)
        {
            if (this->robot_ori_theta >= (this->desire_theta + PI))
            {
                this->robot_ori_theta -= 2 * PI;
            }
        }
        this->heading_err = this->desire_theta - this->robot_ori_theta;
        this->dist_err = this->dist_min;
        this->heading_err_sum += this->heading_err;
        this->dist_err_sum += this->dist_err;

        this->steering_head = (this->p_h * this->heading_err) + (this->i_h * this->heading_err_sum) + (this->d_h * (this->heading_err - this->pre_heading_err) / this->dt);
        this->sterring_ctr = atan2((this->p_c * this->dist_err) + (this->i_h * this->heading_err_sum) + (this->d_h * (this->heading_err - this->pre_heading_err) / this->dt), this->v);
        this->pre_heading_err = this->heading_err;
        this->pre_dist_err = this->dist_err;

        this->desired_robot_vel.linear.x = this->v;
        this->desired_robot_vel.angular.z = this->steering_head + sign * this->sterring_ctr;

         if (this->desired_robot_vel.angular.z > 0.65)
             this->desired_robot_vel.angular.z = 0.65;
         if (this->desired_robot_vel.angular.z < -0.65)
             this->desired_robot_vel.angular.z = -0.65;
        if (arrive_goal)
        {
            this->desired_robot_vel.linear.x = 0;
            this->desired_robot_vel.angular.z = 0;
            ROS_INFO("TRAKING DONE");
            return desired_robot_vel;
        }
        else
        {
            // ROS_INFO("DIST  = %f", this->dist_err);
            // ROS_INFO("DIST AVR = %f", this->dist_err_sum / this->cnt);
        }
        return desired_robot_vel;
    }
};