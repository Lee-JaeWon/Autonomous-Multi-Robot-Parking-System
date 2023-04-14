/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/ui_main/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_main
{

  using namespace Qt;

  /*****************************************************************************
  ** Implementation [MainWindow]
  *****************************************************************************/

  MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
      : QMainWindow(parent), qnode(argc, argv)
  {
    ui.setupUi(this);
    qnode.init();

    QIcon appIcon("/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/ui_main/images/icon.png");
    this->setWindowIcon(appIcon);
  }

  void ui_main::MainWindow::on_btnexit_clicked(){
    qnode.btnExit();
  }

  void ui_main::MainWindow::on_btntaskmark_clicked(){
    qnode.btnMarkerPub();
  }

  void ui_main::MainWindow::on_btngo_clicked(){
    qnode.btnGoalPub();
  }

  void ui_main::MainWindow::on_btnreturn_clicked(){
    qnode.btnReturnPub();
  }

  void ui_main::MainWindow::on_btnposeset_clicked(){
    qnode.Q_btnPoseset();
  }

  MainWindow::~MainWindow() {}

} // namespace ros_qt_format
