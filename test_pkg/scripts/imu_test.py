#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf
import numpy as np
class Imu_test:
    def __init__(self):
        rospy.init_node("imu_test_node")
        rospy.Subscriber("/imu0",Imu,self.ImuCB)
        rospy.spin()
        
    def ImuCB(self,data):
        quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = round(euler[0], 3 )
        pitch = round(euler[1], 3 )
        yaw = round(euler[2], 3 )

        print(np.rad2deg(yaw))
        

if __name__ =="__main__":
    start = Imu_test()
    