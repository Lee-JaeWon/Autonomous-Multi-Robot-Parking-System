/**
 * @file /include/class1_ros_qt_demo/main_window.hpp
 *
 * @brief Qt based gui for class1_ros_qt_demo.
 *
 * @date November 2010
 **/

#ifndef parkingUI_MAIN_WINDOW_H
#define parkingUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include <QtWidgets/QDialog>
#include <QStyleOptionButton>
#include <QTimer>
#include <QPushButton>
#include <QDateTime>
#include <QMessageBox>
#include <filesystem>
#include <list>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QPainter>
#include <QPaintEvent>

#define RED     QColor(255,0,0)
#define GREEN   QColor(0,255,0)
#define BLUE    QColor(0,0,255)
#define YELLOW  QColor(255,255,0)
#define CYAN    QColor(0,255,255)


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace parkingUI {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class ClickableLabel : public QLabel
{
    Q_OBJECT
public:
    explicit ClickableLabel(QWidget* parent = nullptr) : QLabel(parent) {}
    int index=0;
    void setIndex(int i)
    {
      this->index = i;
    }

    void paintEvent(QPaintEvent *event) override
    {
            QLabel::paintEvent(event);

            // 테두리 그리기
            QPainter painter(this);
            painter.setPen(QPen(Qt::black, 1));
            painter.drawRect(QRect(0, 0, width() - 1, height() - 1));
    }

    ParkingInfo Pdata;

signals:
    void clicked(ParkingInfo* Pdata);

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        emit clicked(&Pdata);
        QLabel::mousePressEvent(event);
    }
};

class InfoWindow : public QMainWindow
{
    Q_OBJECT

public:
    InfoWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {
        setWindowTitle("Parkinf Information");
        // 새 창에 내용 추가
        QLabel *label = new QLabel("This is a new window");
        setCentralWidget(label);
    }
};
class NewDialog : public QDialog
{
    Q_OBJECT

public:

  ParkingInfo* parkinginfo;
  QWidget* parent;
  QPalette palette;

  QVBoxLayout *layout_status;
  QLabel *label0,*label1,*label2,*label3;

  QHBoxLayout *layout_button;
  QPushButton* buttonIn;
  QPushButton* buttonOut;
  QPushButton* buttonCancel;

  QString str, str_parkinglot, str_status,
          str_status_kor, str_carNum, str_date;

  //생성자
  explicit NewDialog(ParkingInfo* info, QWidget* parent_) : QDialog(parent)
  {
    this->parkinginfo = info;
    this->parent=parent_;

    SetDialog();
    SetLayout();
    SetConnect();
  }


private:

signals:
    void ParkingIn_SIGNAL(ParkingInfo* parkinginfo);
    void ParkingOut_SIGNAL(ParkingInfo* parkinginfo);

public Q_SLOTS:
    void ButtonInClicked()
    {
      if(label1->text() == "주차상태 : 주차중..")
      {

      }
      emit ParkingIn_SIGNAL(parkinginfo);
    }
    void ButtonOutClicked()
    {
      emit ParkingOut_SIGNAL(parkinginfo);
    }
    void ButtonCancelClicked()
    {
      close();
    }

public :

    void SetInfo(ParkingInfo *info) //외부에서 ParkingInfo 받아와서 업데이트
    {
      this->parkinginfo = info;
      //this->time = QDateTime::currentDateTime();
    }

    void SetBackgroundColor() //배경색설정
    {
      std::string str = this->parkinginfo->GetParkingStatus();
      if(str=="full")
      {
        palette.setColor(QPalette::Background, QColor(255,204,204)); // 배경색을 빨간색으로 설정
        this->setPalette(palette);
      }
      else if(str=="empty")
      {
        palette.setColor(QPalette::Background, QColor(204,255,204)); // 배경색을 파란색으로 설정
        this->setPalette(palette);
      }
      else if(str=="parking")
      {
        palette.setColor(QPalette::Background, QColor(220,220,220)); // 배경색을 파란색으로 설정
        this->setPalette(palette);
      }
    }

    void ParkingIn_ing()
    {
      if(parkinginfo->GetParkingStatus()=="parking")
      {
//        str_status_kor = "주차상태 : ";
//        str_status_kor += "주차중..";

//        label1->setText(str_status_kor);

//        str_carNum = "차량번호 : " + QString::fromStdString(parkinginfo->GetCarNum());
//        label2->setText(str_carNum);

//        str_date = "주차일자 : " + QString::fromStdString(parkinginfo->StartTime());
//        label3->setText(str_date);


        if (layout_status != nullptr)
            {
                QLayoutItem* item;
                while ((item = layout_button->takeAt(0)) != nullptr)
                {
                    QWidget* widget = item->widget();
                    if (widget != nullptr)
                    {
                        layout_button->removeWidget(widget);
                        delete widget;
                    }
                    delete item;
                }
                while ((item = layout_status->takeAt(0)) != nullptr)
                {
                    QWidget* widget = item->widget();
                    if (widget != nullptr)
                    {
                        layout_status->removeWidget(widget);
                        delete widget;
                    }
                    delete item;
                }
                delete layout_status;
            }

        //buttonIn->setEnabled(false);

        SetDialog();
        SetLayout();
        SetConnect();
        update();
      }
    }

    void ParkingIn_done()
    {
      SetBackgroundColor();
      if(parkinginfo->GetParkingStatus()=="full")
      {
        str_status_kor = "주차상태 : ";
        str_status_kor += "주차완료";

        label1->setText(str_status_kor);

        str_carNum = "차량번호 : " + QString::fromStdString(parkinginfo->GetCarNum());
        label2->setText(str_carNum);

        str_date = "주차일자 : " + QString::fromStdString(parkinginfo->StartTime());
        label3->setText(str_date);
      }
    }

    void ParkingOut_ing()
    {
      if(parkinginfo->GetParkingStatus()=="parkout")
      {
        if (layout_status != nullptr)
            {
                QLayoutItem* item;
                while ((item = layout_button->takeAt(0)) != nullptr)
                {
                    QWidget* widget = item->widget();
                    if (widget != nullptr)
                    {
                        layout_button->removeWidget(widget);
                        delete widget;
                    }
                    delete item;
                }
                while ((item = layout_status->takeAt(0)) != nullptr)
                {
                    QWidget* widget = item->widget();
                    if (widget != nullptr)
                    {
                        layout_status->removeWidget(widget);
                        delete widget;
                    }
                    delete item;
                }
                delete layout_status;
            }

        //buttonIn->setEnabled(false);

        SetDialog();
        SetLayout();
        SetConnect();
        update();
      }
    }

    void ParkingOut_done()
    {
      SetBackgroundColor();
      if(parkinginfo->GetParkingStatus()=="empty")
      {
        str_status_kor = "주차상태 : ";
        str_status_kor += "출차완료";

        label1->setText(str_status_kor);

        str_carNum = "차량번호 : ";
        label2->setText(str_carNum);

        str_date = "주차일자 : ";
        label3->setText(str_date);
      }
    }


    void SetDialog()
    {
      //Dialog 설정
      setWindowTitle(QString::number(parkinginfo->GetParkingLot()));
      //setGeometry(1000,1000,500,500);
      setFixedSize(600,600);
      move(parent->geometry().center() - rect().center());

      //위치고정
      setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);

      SetBackgroundColor();
    }

    void SetLayout()
    {
      SetLayOutStatus();
      SetLayOutButton();
      setLayout(layout_status);
    }

    void SetLayOutStatus()
    {
      //Layout 생성
      layout_status = new QVBoxLayout();
      layout_status->setContentsMargins(50, 50, 50, 20);

      //Label 생성
      label0 = new QLabel();
      label1 = new QLabel();
      label2 = new QLabel();
      label3 = new QLabel();

      QFont font = label0->font();
      font.setPointSize(20); // 폰트 크기 조정
      label0->setFont(font);
      label1->setFont(font);
      label2->setFont(font);
      label3->setFont(font);

      str_parkinglot = "주차구역 : PL" + QString::number(parkinginfo->GetParkingLot());
      label0->setText(str_parkinglot);

      str_status = QString::fromStdString(parkinginfo->GetParkingStatus());
      str_status_kor = "주차상태 : ";
      if(str_status=="full")
      {
        str_status_kor += "주차불가";

      }
      else if(str_status=="empty")
      {
        str_status_kor += "주차가능";

      }
      else if(str_status=="parking")
      {
        str_status_kor += "주차중..";

      }
      else if(str_status=="parkout")
      {
        str_status_kor += "출차중..";
      }
      label1->setText(str_status_kor);

      str_carNum = "차량번호 : " + QString::fromStdString(parkinginfo->GetCarNum());
      label2->setText(str_carNum);


      if(str_status=="parkout")
      {
        str_date = "출차일자 : " + QString::fromStdString(parkinginfo->StartTime());
        label3->setText(str_date);
      }
      else {
        str_date = "주차일자 : " + QString::fromStdString(parkinginfo->StartTime());
        label3->setText(str_date);
      }


      layout_status->addWidget(label0);
      layout_status->addWidget(label1);
      layout_status->addWidget(label2);
      layout_status->addWidget(label3);
    }

    void SetLayOutButton()
    {
      //Layout 생성
      layout_button = new QHBoxLayout();
      layout_button->setContentsMargins(50, 50, 50, 50);

      //Button 생성
      buttonIn = new QPushButton("주차하기");
      buttonOut = new QPushButton("출차하기");
      buttonCancel = new QPushButton("닫기");

      //Style of buttons
      QFont font_("Arial", 20);

      buttonIn->setFixedSize(150, 100);
      buttonIn->setFont(font_);
      buttonIn->setChecked(false);

      buttonOut->setFixedSize(150, 100);
      buttonOut->setFont(font_);
      buttonOut->setChecked(false);

      buttonCancel->setFixedSize(150, 100);
      buttonCancel->setFont(font_);
      buttonCancel->setChecked(false);

      if(str_status=="empty")
      {

        layout_button->addWidget(buttonIn);

      }
      else if(str_status=="full")
      {

        layout_button->addWidget(buttonOut);

      }

      layout_button->addSpacing(20); // 간격 설정
      layout_button->addWidget(buttonCancel);
      layout_status->addLayout(layout_button);
    }

    void SetConnect()
    {
      connect(buttonIn,&QPushButton::clicked, this,&NewDialog::ButtonInClicked);
      connect(buttonOut,&QPushButton::clicked, this,&NewDialog::ButtonOutClicked);
      connect(buttonCancel,&QPushButton::clicked, this,&NewDialog::ButtonCancelClicked);
    }
};


class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  //Variables
  QPixmap map_ori;
  QPixmap map;

  //now Target
  std::vector<double> parkingLotTarget;
  QGraphicsView* parkingLotTarget_view;
  QLabel* parkingLotTarget_label;
  ParkingInfo* parkingLotTarget_info;

  //now Out Target
  std::vector<double> parkoutLotTarget;
  QGraphicsView* parkoutLotTarget_view;
  QLabel* parkoutLotTarget_label;
  ParkingInfo* parkoutLotTarget_info;

  std::list<int> EmptyList;
  int parkingNum; //nums of parking lot
  ClickableLabel* PL;
  QLabel* InputLot;
  QLabel* OutputLot;
  InfoWindow *infoWindow = nullptr;
  NewDialog * newDialog = nullptr;

  void SetLabelGreen(QLabel* l);
  void SetLabelRed(QLabel* l);
  void SetLabelGray(QLabel* l);

  std::vector<double> TransXY(std::vector<double> point);

  void ParkingLotInit();
  void InOutLotInit();

  void UpdateTargetPose(int num, float x, float y); //, QPixmap* pixmap);

  void ParkingIn_Ready();
  void ParkingIn_NotReady();
  void ParkingOut_Ready();
  void ParkingOut_NotReady();

  bool canPark = true;


public Q_SLOTS:

  void on_pushButton_ParkIn_clicked();
  void on_pushButton_ParkOut_clicked();

  void RobotPose_SLOT(nav_msgs::Odometry::ConstPtr odom);
  void ParkingDone_SLOT(parking_msgs::parkingDone::ConstPtr data);
  void ParkoutDone_SLOT(parking_msgs::parkingDone::ConstPtr data);
  void Sequence_SLOT(parking_msgs::Sequence::ConstPtr seq);

  // Second Window에서 오는 시그널을 받는 SLOT
  void ParkingIn_SLOT(ParkingInfo* info);
  void ParkingOut_SLOT(ParkingInfo* info);

  // For second Window(parkinglot information)
  void openNewWindow(ParkingInfo* info);
  void openNewWindow(int num);
  void onParkingLabelClicked(ParkingInfo* info);


private:
//public:
	Ui::MainWindowDesign ui;
	QNode qnode;


};

}  // namespace parkingUI



#endif // parkingUI_MAIN_WINDOW_H
