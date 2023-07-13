#!/usr/bin/env python3

# import libraries
import rospy
import cv2
from parking_msgs.srv import carNum ,carNumResponse,carNumRequest
import numpy as np
import car_plate_number_test
import sensor_msgs
from cv_bridge import CvBridge
import cv2

class Number_Detect:
    def __init__(self):
        print("Start")
        rospy.init_node("car_plate_number")
        rospy.Subscriber("/usb_cam/image_raw",sensor_msgs.msg.Image,self.ImageCallback)
        self.service = rospy.Service('carNum_srv',carNum,self.car_number_handle)
        self.image = None
        self.flag = False
        self.bridge = CvBridge()
        rospy.spin()

    def car_number_handle(self,req):
        detect = False
        rospy.loginfo("service called")
        data = carNumResponse()
        while (not detect): 
            
            car_number , img ,detected= car_plate_number_test.find_number(self.image)
            if detected:
                detect = True
                rospy.loginfo("Object Detected!")
                data.carExist = detected
                data.data = self.bridge.cv2_to_imgmsg(img).data
                data.carNum = car_number
                return data
            else:
                data.carExist = detected
                data.data = []
                data.carNum = ""
                return False


    # def start(self,data):
    #     print("success")
    #     car_number , img ,detected= car_plate_number_test.find_number(self.image)
    #     # print(car_number)
    #     carmsg = CarData()
    #     if detected:
    #         carmsg.car_number_plate = car_number
    #         carmsg.height = img.shape[0]
    #         carmsg.width = img.shape[1]
    #         carmsg.data = self.bridge.cv2_to_imgmsg(img).data
    #         carmsg.detected = detected
    #         self.car_msg_pub.publish(carmsg)
    #         rospy.loginfo("{} DETECTED SUCCESS".format(car_number))
    #     else:
    #         carmsg.car_number_plate = " "
    #         carmsg.detected = detected
    #         self.car_msg_pub.publish(carmsg)
    #         rospy.loginfo("DETECTED FAILED")



# 집에 가고 싶다...

    def ImageCallback(self, data):
        # rospy.loginfo("image received ")
        # print(data.height) # value : 1080
        # print(data.width) # value : 1920
        # print(data.encoding) # value : rgb8
        # print(data.is_bigendian) # value :  0
        # print(data.step) # value : 5760
        # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        img_ori = self.bridge.imgmsg_to_cv2(data,'bgr8')
        # self.image = cv2.resize(img_ori, (640, 480), interpolation=cv2.INTER_CUBIC)
        self.image = img_ori

            
if __name__ =="__main__":
    start = Number_Detect()
    