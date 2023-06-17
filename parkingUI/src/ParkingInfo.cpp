#include<iostream>
#include<fstream>
#include<string>
#include <std_msgs/String.h>
//#include <QDateTime>
#include <ctime>


class ParkingInfo
{
  private :

    int parkingLot;
    std::string status;
    std::string carNum;

    int startYear;
    int startMonth;
    int startDate;
    int startHour;
    int startMinute;

    int endYear;
    int endMonth;
    int endDay;
    int endHour;
    int endMinute;


  public :

    // Getter
    int GetParkingLot()
    {
      return this->parkingLot;
    }

    std::string GetParkingStatus()
    {
      return this->status;
    }

    std::string GetCarNum()
    {
      return this->carNum;
    }

    std::string StartTime()
    {
      std::string str;
      str+= std::to_string(this->startYear);
      str+= "/";
      str+= std::to_string(this->startMonth);
      str+= "/";
      str+= std::to_string(this->startDate);
      str+= " ";
      str+= std::to_string(this->startHour);
      str+= ":";
      str+= std::to_string(this->startMinute);

      return str;
    }

    int GetYear() {return startYear;}
    int GetMonth() {return startMonth;}
    int GetDate() {return startDate;}
    int GetHour() {return startHour;}
    int GetMinute() {return startMinute;}


    // Setter

    void SetCarNum(std::string carNum)
    {
      this->carNum = carNum;
    }

    void SetStatus(std::string status)
    {
      this->status=status;
    }

    void SetEmptyTime()
    {
      this->startYear=0;
      this->startMonth=0;
      this->startDate=0;
      this->startHour=0;
      this->startMinute=0;
    }

    void SetStartTime()
    {
      time_t timer;
      struct tm* t;
      timer = time(NULL);
      t = localtime(&timer);

      this->startYear=t->tm_year+1900;
      this->startMonth=t->tm_mon+1;
      this->startDate=t->tm_mday;
      this->startHour=t->tm_hour;
      this->startMinute=t->tm_min;
    }

    void SetOutTime(int Year, int Mon, int Date, int Hour, int Min)
    {
      this->endYear = Year;
    }

    void SetDataInit(int parkingLot, std::string status, std::string carInfo ,int Year, int Mon, int Date, int Hour, int Min)
    {
      this->parkingLot = parkingLot;
      this->status = status;
      this->carNum = carInfo;
      this->startYear = Year;
      this->startMonth = Mon;
      this->startDate = Date;
      this->startHour = Hour;
      this->startMinute = Min;
    }

    void SetDataInit(int parkingLot, std::string status)
    {
      this->parkingLot = parkingLot;
      this->status = status;
    }

    // Print Information
    void PrintInfo()
    {
      std::cout<<"-------------------------------- \n";
      std::cout<<" parkingLot : "<< parkingLot <<"\n";
      std::cout<<" status : "<< status <<"\n";
      std::cout<<" carInfo : "<< carNum <<"\n";
      std::cout<<" date : "<< startYear << "/" << startMonth << "/" << startDate << "/" <<startHour << "/" << startMinute <<"\n";
    }
};
