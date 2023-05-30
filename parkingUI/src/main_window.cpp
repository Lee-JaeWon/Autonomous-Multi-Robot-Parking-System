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
#include <QGraphicsItem>
#include "../include/parkingUI/main_window.hpp"

#define UI_OFFSET_X 10
#define UI_OFFSET_Y 40

#define PARKING_WIDTH 52
#define PARKING_HEIGHT 52

#define MAP_RE_WIDTH 1024
#define MAP_RE_HEIGHT 1024

enum {PARKIN, PARKOUT};

/*****************************************************************************
** Namespaces
*****************************************************************************/

//#define

namespace parkingUI {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
  ui.setupUi(this);
  qnode.init();
  this->parkingNum=qnode.parkingNum;
  this->EmptyList=qnode.EmptyList;

  // You have to change user's name : hyedo->???
  QString img_path = "/home/hyedo/map.pgm";
  //QString img_path = "/home/lee-jaewon/catkin_ws/src/Autonomous-Multi-Robot-Parking-System/multi_parking_sys/map/mymap0527.pgm";
  QImage img(img_path);

  // Load Image
  map_ori = QPixmap::fromImage(img);

  // Rotate Image 90 degree (CCW)
  //map = map.transformed(QTransform().rotate(-90));

  #define MAP_WIDTH map_ori.width()
  #define MAP_HEIGHT map_ori.height()
  #define MAP_RESOLUTION qnode.map_resolution

  map = map_ori.scaled(MAP_RE_WIDTH,MAP_RE_HEIGHT);

  // Show map Image on graphicsView
  QGraphicsScene* scene = new QGraphicsScene;
  ui.graphicsView->setScene(scene);
  scene->addPixmap(map);

  //SIGNAL//
  qRegisterMetaType<nav_msgs::Odometry::ConstPtr>("nav_msgs::Odometry::ConstPtr");
  QObject::connect(&qnode,SIGNAL(RobotPose_SIGNAL(nav_msgs::Odometry::ConstPtr)), this, SLOT(RobotPose_SLOT(nav_msgs::Odometry::ConstPtr)));

  qRegisterMetaType<std_msgs::Bool::ConstPtr>("std_msgs::Bool::ConstPtr");
  QObject::connect(&qnode,SIGNAL(ParkingDone_SIGNAL(std_msgs::Bool::ConstPtr)), this, SLOT(ParkingDone_SLOT(std_msgs::Bool::ConstPtr)));

  //INIT
  ParkingLotInit();


  // TEST  //

//  ui.unparking_button->setStyleSheet("background-color: rgba(0,0,0,0);");
//  ui.unparking_button->setStyleSheet("color: rgba(0,0,0,0);");
//  ui.unparking_button->setStyleSheet("border: none;");
//  ui.unparking_button->setText("");
//  QDialog* d = new QDialog();
//  QVBoxLayout *layout = new QVBoxLayout();
//  QLabel* lab = new QLabel();
//  layout->alignment(centralWidget(lab));
//  layout->addWidget(lab);
//  layout.setAlignment(lab,center);
//  layout.set
}


  //----------- Methods -----------//

  // Make ParkingLot Labels and setStyle(location, size, color)
  void MainWindow::ParkingLotInit()
  {
    QGraphicsScene* scene = ui.graphicsView->scene();

    PL = new ClickableLabel[parkingNum];
    for(int i=0; i<parkingNum; i++)
    {
      PL[i].setIndex(i);
      PL[i].Pdata = qnode.ParkingData[i];

      QString text = "PL" + QString::number(i);
      PL[i].setText(text);

      std::vector<double> point = TransXY(qnode.PL[i]);
      PL[i].setGeometry(point.at(0),point.at(1),PARKING_WIDTH,PARKING_HEIGHT);

      PL[i].setAlignment(AlignCenter);

      if(PL[i].Pdata.GetParkingStatus() == "empty") SetLabelGreen(&PL[i]);
      //else SetLabelRed(&PL[i]);
      else SetLabelGray(&PL[i]);
      scene->addWidget(&PL[i]);

      connect(&PL[i], &ClickableLabel::clicked, this, &MainWindow::onParkingLabelClicked);
    }
  }

  // Callback robot's pose and plot
  void MainWindow::RobotPose_SLOT(nav_msgs::Odometry::ConstPtr odom)
  {
    float y = -odom->pose.pose.position.x;
    float x = -odom->pose.pose.position.y;

    // Location Scaling
    x = x * MAP_RE_WIDTH / (MAP_WIDTH * MAP_RESOLUTION);
    y = y * MAP_RE_HEIGHT / (MAP_HEIGHT * MAP_RESOLUTION);
    x += 9.2 / (MAP_WIDTH * MAP_RESOLUTION) * MAP_RE_WIDTH;
    y += 9.2 / (MAP_HEIGHT * MAP_RESOLUTION) * MAP_RE_HEIGHT;


    // Draw Point
    int frameNum = odom->header.frame_id[5]-'0';
    UpdateTargetPose(frameNum,x,y);
  }

  // Callback is current parking mission done
  void MainWindow::ParkingDone_SLOT(std_msgs::Bool::ConstPtr data)
  {
    if(!data->data) return;

    SetLabelRed(parkingLotTarget_label);
    parkingLotTarget_info->SetStatus("full");

    newDialog->SetInfo(parkingLotTarget_info);
    newDialog->ParkingIn_done();
    PL[parkingLotTarget_info->GetParkingLot()].Pdata=*parkingLotTarget_info;
    qnode.ParkingData[parkingLotTarget_info->GetParkingLot()] = *parkingLotTarget_info;
    qnode.WriteParkingData();
  }

  // Second Window에서 오는 주차 실행 시그널을 받는 SLOT
  void MainWindow::ParkingIn_SLOT(ParkingInfo* info)
  {
    bool isCar=true;

    parking_msgs::carNum service;

    //if(isCar)
    if(qnode.clientCarNum.call(service))
    {
      QMessageBox msgBox;
      QString str1 = "차량번호가 올바른지 확인해주세요";
      QString str2 = "차량번호 : ";
      str2 += "12가3456";

      msgBox.setWindowTitle("차량번호 확인 메시지");
      msgBox.setText(str1);
      msgBox.setInformativeText(str2);
      msgBox.setStandardButtons(QMessageBox::No|QMessageBox::Yes);
      msgBox.setDefaultButton(QMessageBox::Yes);
      int ret = msgBox.exec();

      if(ret==QMessageBox::Yes)
      {

        parking_msgs::order srv;
        srv.request.order = PARKIN;
        srv.request.parkinglot = info->GetParkingLot();
        srv.request.carNum = "12가3456";

        if(qnode.client.call(srv))
        {
          std::cout<<"Successed Service!!"<<"\n";
          std::cout<<"carNum : "<<srv.response.answer<<"\n";
          std::cout<<srv.response.accepted<<"\n";

          std::vector<double> target;
          int index = info->GetParkingLot();
          target = qnode.PL[index];

          info->SetCarNum("12가3456");
          info->SetStartTime();
          info->SetStatus("parking");

          parkingLotTarget_label = &PL[index];
          parkingLotTarget_info = info;
          parkingLotTarget = target;


          //qnode.ParkingGoalPublsiher(target);
          std::cout<<"Published!!! \n";
          newDialog->SetInfo(info);
          newDialog->ParkingIn_ing();
        }
        else
        {
          std::cout<<"Failed Service!!"<<"\n";
        }
      }
      else if(ret==QMessageBox::No)
      {

      }
      else {

      }
    }
    else
    {
      QMessageBox msgBox;
      QString str1 = "차량번호가 인식되지 않았습니다.";
      QString str2 = "차량을 올바르게 위치해주세요.";

      msgBox.setIcon(QMessageBox::Information);
      msgBox.setWindowTitle("차량번호 확인 불가");
      msgBox.setText(str1);
      msgBox.setInformativeText(str2);
      msgBox.setStandardButtons(QMessageBox::Ok);
      msgBox.exec();

    }


  }

  void MainWindow::UpdateTargetPose(int num, float x, float y)  //, QPixmap* pixmap)
  {
    // item-> data로 robot1,2,3 구분
    QGraphicsScene* scene = ui.graphicsView->scene();
    QList<QGraphicsItem*> items = scene->items();
    for (QGraphicsItem* item : items) {
      if (item->type() == (QGraphicsEllipseItem::Type && item->data(num)==num)) {
        scene->removeItem(item);
        delete item;
      }
    }

    // QGraphicsEllipseItem 생성 및 추가
    int radius = 6;  // 반지름 설정
    QGraphicsEllipseItem* ellipseItem = new QGraphicsEllipseItem(x - radius, y - radius, radius * 2, radius * 2);
    QColor color;
    switch(num%5)
    {
    case 0:
      color = RED;
      break;
    case 1:
      color = GREEN;
      break;
    case 2:
      color = BLUE;
      break;
    case 3:
      color = YELLOW;
      break;
    case 4:
      color = CYAN;
      break;
    }
    ellipseItem->setPen(QPen(Qt::black, 1, Qt::SolidLine));
    ellipseItem->setBrush(QBrush(color));
    QVariant value = num;
    ellipseItem->setData(num,value);
    scene->addItem(ellipseItem);

    //Label 생성 및 추가
    QString itemName = "Robot_"+QString::number(num);
    QGraphicsTextItem* textItem = new QGraphicsTextItem(itemName);
    //Label Font 설정
    QFont font;
    font.setPointSize(8);
    font.setBold(false);
    textItem->setFont(font);
    textItem->setPos(x-20,y+10);
    scene->addItem(textItem);

    // pixmap을 QGraphicsScene에 추가
    //QGraphicsPixmapItem* pixmapItem = new QGraphicsPixmapItem(*pixmap);
    //scene->addItem(pixmapItem);
  }

  void MainWindow::openNewWindow(ParkingInfo* info)
  {
    delete newDialog;
    newDialog = new NewDialog(info,this);
    //newDialog->exec();
    newDialog->show();
    connect(newDialog, &NewDialog::ParkingIn_SIGNAL, this, &MainWindow::ParkingIn_SLOT);
  }

  void MainWindow::openNewWindow(int num)
  {
    delete newDialog;
    newDialog = new NewDialog(&qnode.ParkingData[num],this);
    newDialog->show();
    connect(newDialog, &NewDialog::ParkingIn_SIGNAL, this, &MainWindow::ParkingIn_SLOT);
  }

  MainWindow::~MainWindow() {}


}  // namespace parkingUI

std::vector<double> parkingUI::MainWindow::TransXY(std::vector<double> point)
{
  std::vector<double> result = point;

  result.at(0) = (-point.at(1)+9.2)*MAP_RE_WIDTH/(MAP_WIDTH * MAP_RESOLUTION);
  result.at(0) -= PARKING_WIDTH/2;

  result.at(1) = (-point.at(0)+9.2)*MAP_RE_HEIGHT/(MAP_HEIGHT * MAP_RESOLUTION);
  result.at(1) -= PARKING_HEIGHT/2;

  return result;
}

void parkingUI::MainWindow::SetGraphicGreen(QGraphicsView* v)
{
  v->setStyleSheet("background-color: rgba(204,255,204,80);");
}

void parkingUI::MainWindow::SetGraphicRed(QGraphicsView* v)
{

  v->setStyleSheet("background-color: rgba(255,204,204,200);");
}

void parkingUI::MainWindow::SetLabelGreen(QLabel* l)
{
  l->setStyleSheet("background-color: rgba(204,255,204,100);");
}

void parkingUI::MainWindow::SetLabelRed(QLabel* l)
{
  l->setStyleSheet("background-color: rgba(255,204,204,180);");
}

void parkingUI::MainWindow::SetLabelGray(QLabel* l)
{
  l->setStyleSheet("background-color: rgba(220,220,220,180);");
}

void parkingUI::MainWindow::onParkingLabelClicked(ParkingInfo* info)
{
  //선택한 주차공간 채우기
  openNewWindow(info);
  //std::cout<<"num is : "<< info->GetParkingLot() <<std::endl;
}

void parkingUI::MainWindow::on_pushButton_ParkIn_clicked()
{
  //가장 가까운 주차공간부터 채우기
  openNewWindow(&qnode.ParkingData[*std::min_element(EmptyList.begin(), EmptyList.end())]);
}

void parkingUI::MainWindow::on_pushButton_ParkOut_clicked()
{

}
