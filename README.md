# Autonomous-Multi-Robot-Parking-System

This is the Repository of the 2023 Capstone Design 'Multi in Your Front yard' team in the Department of Robotics at Kwangwoon University.<br>
Development Period: 2023.3 to 2023.6

## About
Please read the [abstract.md file](./abstract.md) of our study.

## Member

|Member|Role|
|:---:|:---:|
|[Chunggil An(팀장)](https://github.com/chunggilan)|Global Path Planning, Dynamic Obstacle Avoidance, Path Tracking|
|[Jaewon Lee](https://github.com/Lee-JaeWon)|Multi Robot System, Localization, Hardware|
|[Hyedo Kim](https://github.com/KIM-HYEDO)|Multi Robot System, Path Tracking, Localization, GUI|
|[Hyoseok Joo](https://github.com/JooHyoSeok)|Global Path Planning, Path Tracking, Collision Avoidance|  

## About Setting
[About_Setting.md](./About_Setting/)<br>
This is a ROS Noetic environment. We mainly used C++, and we didn't want to use move_base. Except for AMCL packages and motor-related packages(Dynamixel), it was implemented almost manually.  
There can be various problems to apply recklessly (because it is applied to a robot made by itself). It would be good to carefully examine the code and utilize this repository.  
It can also be tested purely on simulation. It will be organized and uploaded soon.

## Result
<p align="center"><img src="./gif/parkin720.gif" width="500px"></p>  
<p align="center"> Park In </p>   
<p align="center"><img src="./gif/parkout720.gif" width="500px"></p>  
<p align="center"> Park Out </p>   
<p align="center"><img src="./gif/parkinout720.gif" width="500px"></p>  
<p align="center"> Park In&Out </p>

## Paper
[이재원, 주효석, 안충길, 김혜도, and 오정현 , "다중 자율 로봇 주차 시스템," in 제어로봇시스템학회 국내학술대회 논문집, 2023, pp. 600-601.](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11480481)
