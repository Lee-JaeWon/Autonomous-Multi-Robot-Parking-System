#!/usr/bin/env python3

from parking_msgs.srv import carNum ,carNumResponse,carNumRequest
from test_pkg.msg import CarData
import rospy
from sensor_msgs.msg import Image

import car_plate_number_test
from cv_bridge import CvBridge

def car_number_handle(req):
    print(req)
    data = carNumResponse()
    bridge = CvBridge()
    detected = False

    car_number , img ,detected= car_plate_number_test.find_number()
    if detected:
        data.carExist = True
        data.data = bridge.cv2_to_imgmsg(img).data
        data.carNum = car_number
        return data
    else:
        return

def car_number_plate_server():
    rospy.init_node("car_number_plate_server")
    service = rospy.Service('carNum_srv',carNum,car_number_handle)
    rospy.spin()

if __name__ == "__main__":
    car_number_plate_server()




