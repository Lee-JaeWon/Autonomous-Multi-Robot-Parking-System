#include <test_pkg/local_goal_pub.h>



void OdomCB(const nav_msgs::Odometry::ConstPtr& msg){

    cur_rx = msg -> pose.pose.position.x;
    cur_ry = msg -> pose.pose.position.y;

    if (Path_Sub){

        int cur_idx = calc_current_index(cur_rx,cur_ry,global_path);
        int tar_idx = calc_target_index(global_path.poses[cur_idx].pose.position.x,global_path.poses[cur_idx].pose.position.y,global_path, cur_idx);
        double target_x = global_path.poses[tar_idx].pose.position.x;
        double target_y = global_path.poses[tar_idx].pose.position.y;
        geometry_msgs::Quaternion target_quat = global_path.poses[tar_idx].pose.orientation;
        local_goal.header.frame_id = "map";
        local_goal.pose.position.x = target_x;
        local_goal.pose.position.y = target_y;
        local_goal.pose.orientation = target_quat;

        local_goal_pub.publish(local_goal);
    }
}

void PathCB(const nav_msgs::Path::ConstPtr& msg){

    std::cout<<"Path Subscribed"<< std::endl;
    global_path = *msg;
    Path_Sub = true;

}


int calc_current_index(double x , double y ,const nav_msgs::Path& gPath ){

    PathSize = gPath.poses.size();
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
    
    if (targetIdx == PathSize){
        Path_Sub = false;
        std::cout << "Goal_reached" << std::endl;
    }
    return targetIdx;

}


int main(int argc, char **argv)// 노드메인함수

{
    ros::init(argc, argv, "path_node");
    ros::NodeHandle nh; 
    odom_sub = nh.subscribe("/odom",100,OdomCB);
    gpath_sub = nh.subscribe("/path",100,PathCB);
    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("local_goal",1);
    std::cout << "start" <<std::endl;
    ros::spin();

    return 0;
}