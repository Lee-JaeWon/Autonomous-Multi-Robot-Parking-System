#!/usr/bin/env python3
# import libraries
import rospy
import cv2
import numpy as np
import car_plate_number_test
import sensor_msgs
from geometry_msgs.msg import PoseStamped
from test_pkg.msg import CarData
from cv_bridge import CvBridge
import cv2

class License_Detect:
    def __init__(self):
        print("Start")
        rospy.init_node("car_plate_number")
        rospy.Subscriber("/camera/rgb/image_raw",sensor_msgs.msg.Image,self.ImageCallback)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.start)
        self.car_msg_pub = rospy.Publisher('/Car_Data',CarData,queue_size=1)
        self.image = None
        self.oldval = 0
        self.bridge = CvBridge()
        rospy.spin()
    
    def start(self,data):
        print("success")
        car_number , img ,detected= car_plate_number_test.find_number()
        # print(car_number)

        carmsg = CarData()

        if detected:
            carmsg.car_number_plate = car_number
            carmsg.height = img.shape[0]
            carmsg.width = img.shape[1]
            carmsg.data = self.bridge.cv2_to_imgmsg(img).data
            carmsg.detected = detected
            self.car_msg_pub.publish(carmsg)
        else:
            carmsg.car_number_plate = " "
            carmsg.detected = detected

            self.car_msg_pub.publish(carmsg)


# 집에 가고 싶다...

    def ImageCallback(self, data):
        # print(data.height) # value : 1080
        # print(data.width) # value : 1920
        # print(data.encoding) # value : rgb8
        # print(data.is_bigendian) # value :  0
        # print(data.step) # value : 5760
        bridge = CvBridge()
        # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
        img_ori = bridge.imgmsg_to_cv2(data,'bgr8')
        # print(img_ori.shape)
        self.image = cv2.resize(img_ori, (640, 480), interpolation=cv2.INTER_CUBIC)
        # self.image = img_ori.copy()
        # print(self.image[0][0]) # row : 1080 col : 1920
        # print(np.sum(self.image))
        cur = np.sum(self.image)
        print(cur - self.oldval if cur > self.oldval else 0)
        self.oldval = cur



        cv2.imshow("Image window", self.image)
        cv2.waitKey(3)
        # print(car_plate_number_test.find_number(data))
    
            
if __name__ =="__main__":
    start = License_Detect()
    