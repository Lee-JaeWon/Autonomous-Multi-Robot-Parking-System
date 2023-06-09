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

namespace ui_emergency
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
    void btnClicked();
    void btnExit();
    void btnInitPose();
    int count = 0;
    bool emer_btn_flag = false;
    bool poseinit_flag = false;
    double robot_1_x;
    double robot_1_y;
    double robot_1_a;
    double robot_2_x;
    double robot_2_y;
    double robot_2_a;
    double robot_3_x;
    double robot_3_y;
    double robot_3_a;

  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char **init_argv;

    ros::Publisher chatter_publisher;
    ros::Publisher chatter_emer;
    ros::Publisher initpose_one;
    ros::Publisher initpose_two;
    ros::Publisher initpose_thr;
  };

} // namespace ui_emergency

#endif /* ui_emergency_QNODE_HPP_ */
