#ifndef ui_emergency_QNODE_HPP_
#define ui_emergency_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

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

    int count = 0;
    
    bool btn_marker_pub_flag = false;
    bool btn_goal_flag = false;
    bool btn_return_flag = false;

  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char **init_argv;


    ros::Publisher vis_pub;
    ros::Publisher vis_pub_text;
    ros::Publisher vis_way;
    ros::Publisher goal_pub;
    ros::Publisher return_pub;
  };

} // namespace ui_main

#endif /* ui_emergency_QNODE_HPP_ */
