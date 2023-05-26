#include <test_pkg/local_goal_pub.h>
#include <iostream>

std::string s;
std::string ns;

//  return euclidean distance
double calc_dist(double x , double y , double nx  , double ny){
    return sqrt(pow(x-nx,2)+pow(y-ny,2));
}
// Odometry_subscribe and calc_local_goal
void OdomCB(const geometry_msgs::PoseStamped::ConstPtr& msg){

    cur_rx = msg -> pose.position.x;
    cur_ry = msg -> pose.position.y;
    double target_x,target_y ,target_nx ,target_ny;
    geometry_msgs::Quaternion target_quat;

    if (Path_Sub){

        int cur_idx = calc_current_index(cur_rx,cur_ry,global_path);
        int tar_idx = calc_target_index(global_path.poses[cur_idx].pose.position.x ,global_path.poses[cur_idx].pose.position.y,global_path, cur_idx);

        if (tar_idx == PathSize){

            target_x = global_path.poses[tar_idx].pose.position.x;
            target_y = global_path.poses[tar_idx].pose.position.y;
            target_quat = calc_target_quat(global_path.poses[tar_idx -1].pose.position.x ,global_path.poses[tar_idx -1].pose.position.y , target_x, target_y);
            
            double dist = calc_dist(cur_rx,cur_ry,target_x ,target_y);
            if (dist < 0.1) {
                ROS_INFO("FINISHED");
                Path_Sub = false;
                return;
            }
        }

        else {
            target_x = global_path.poses[tar_idx].pose.position.x;
            target_y = global_path.poses[tar_idx].pose.position.y;

            target_nx = global_path.poses[tar_idx + 1].pose.position.x;
            target_ny = global_path.poses[tar_idx + 1].pose.position.y;

            //  calculate quat between target_x , target_next_x
            target_quat = calc_target_quat(target_x , target_y , target_nx, target_ny);

        }

        local_goal.header.frame_id = "robot_1/map";
        local_goal.pose.position.x = target_x;
        local_goal.pose.position.y = target_y;
        local_goal.pose.orientation = target_quat;


        // publish local_goal topic 

        local_goal_pub.publish(local_goal);
    }
}
// calc quaternion 
geometry_msgs::Quaternion calc_target_quat(double cx, double cy , double nx, double ny){
    
    geometry_msgs::Quaternion target_quat;
    tf2::Quaternion quaternion;

    double diff_x = nx - cx ;
    double diff_y = ny - cy ;
    double diff_angle = std::atan2(diff_y , diff_x);
    

    quaternion.setRPY(0,0,diff_angle);

    target_quat.x = quaternion.x();
    target_quat.y = quaternion.y();
    target_quat.z = quaternion.z();
    target_quat.w = quaternion.w();


    return target_quat;

}
// Path Callback method 
void PathCB(const nav_msgs::Path::ConstPtr& msg){

    std::cout<<"Path Subscribed"<< std::endl;
    global_path = *msg;
    PathSize = msg->poses.size() -1 ;
    Path_Sub = true;

}

// calc current index by shortest distance and return index
int calc_current_index(double x , double y ,const nav_msgs::Path& gPath ){

    double minDist = 1e6;
    int minIdx = 0;
    int cnt = 0;
    double dist;
    for(auto data : gPath.poses){
        dist = sqrt(pow(data.pose.position.x - x,2) + pow(data.pose.position.y - y,2));
        if (dist < minDist){
            minDist = dist;
            minIdx = cnt;
        }
        cnt++;
    }

    return minIdx;
}
// find nearest index target_distance away and return index 
int calc_target_index(double mx, double my , const nav_msgs::Path& gPath, int curIdx){
    double minDiff = 1e6;
    int targetIdx = 0;
    int cnt = 0;
    double diff;

    for(auto data : gPath.poses){
        diff = fabs(sqrt(pow(data.pose.position.x - mx,2) + pow(data.pose.position.y - my,2)) - target);
        if (diff < minDiff &&  cnt > curIdx){
            minDiff = diff;
            targetIdx = cnt;
        }
        cnt++;
    }
    
    // if (targetIdx == PathSize){
    //     Path_Sub = false;
    //     std::cout << "Goal_reached" << std::endl;
    // }
    return targetIdx;

}


int main(int argc, char **argv)// 노드메인함수

{
    ros::init(argc, argv, "path_node");
    ros::NodeHandle nh; 

    if (ros::param::get("~namespace", s))
        ROS_INFO("local_goal_pub node got param: %s", s.c_str());
    else
        ROS_ERROR("local_goal_pub Failed to get param '%s'", s.c_str());

    ns = '/' + s;

    odom_sub = nh.subscribe(ns + "/tracked_pose",100,OdomCB);
    gpath_sub = nh.subscribe(ns + "/global_path",100,PathCB);
    // gpath_sub = nh.subscribe("spline_path",100,PathCB);
    // gpath_sub = nh.subscribe("/right_path",100,PathCB);


    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>(ns + "/local_goal",1);
    std::cout << "start" <<std::endl;
    ros::spin();

    return 0;
}