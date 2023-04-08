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
#include "../include/ui_emergency/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ui_emergency
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
  }

  void ui_emergency::MainWindow::on_pushButton_clicked()
  {
    qnode.btnClicked();
  }

  void ui_emergency::MainWindow::on_btnexit_clicked(){
    qnode.btnExit();
  }

  MainWindow::~MainWindow() {}

} // namespace ros_qt_format
