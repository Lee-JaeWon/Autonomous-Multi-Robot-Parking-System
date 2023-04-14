#ifndef ui_emergency_QNODE_HPP_
#define ui_emergency_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <geometry_msgs/PoseStamped.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_main
{

  /*****************************************************************************
  ** Class
  *****************************************************************************/

  class QNode : public QThread
  {
    Q_OBJECT
  public:
    QNode(int argc, char **argv);
    virtual ~QNode();
    bool init();
    void run();
    
    void btnExit();
    void btnMarkerPub();
    void btnGoalPub();
    void btnReturnPub();
    void Q_btnPoseset();

    int count = 0;
    
    // coord frame
    std::string robot_1_map = "robot_1/map";
    /////////////////////////////////////////////
    
    // flags
    bool btn_marker_pub_flag = false;
    bool btn_goal_flag = false;
    bool btn_return_flag = false;
    bool btn_poseset_flag = false;

    bool stop_flag = true;
    bool rotate_flag = true;
    bool range_flag = true;
    bool dist_flag = false;
    /////////////////////////////////////////////

    // robot_1::Tracked Pose
    std::string track_ns_robot1 = "/robot_1/tracked_pose";
    double roll, pitch, yaw;
    double roll2, pitch2, yaw2;
    /////////////////////////////////////////////

    // goal, pre_goal, text
    double goal_x = 1.8;
    double goal_y = 1.6;
    double goal_ox = 0.0;
    double goal_oy = 0.0;
    double goal_oz = 0.69862;
    double goal_ow = 0.71548;

    double pre_goal_x = 1.8;
    double pre_goal_y = 1.1;
    double pre_goal_oz = 0.69862;
    double pre_goal_ow = 0.71548;

    double text_first_goal_x = 1.8;
    double text_first_goal_y = 1.8;
    double text_first_goal_oz = 0.69862;
    double text_first_goal_ow = 0.71548;
    /////////////////////////////////////////////

  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char **init_argv;

    // Publisher
    ros::Publisher vis_pub;
    ros::Publisher vis_pub_text;
    ros::Publisher vis_way;
    ros::Publisher goal_pub;
    ros::Publisher return_pub;
    ros::Publisher pub_vel;

    ros::Publisher cmd_stop_pub_1;
    /////////////////////////////////////////////

    // Subscriber
    ros::Subscriber tracked_sub;
    /////////////////////////////////////////////
  };

} // namespace ui_main

#endif /* ui_emergency_QNODE_HPP_ */
