#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy,tf
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
class Path_test():
    def __init__(self):
        rospy.init_node("path_node")
        rospy.Subscriber("/local_planner/dwa_planner/selected_trajectory",Marker,self.callback)
        rospy.Subscriber("odom",Odometry,self.OdomCB)
        self.rx = 0
        self.ry = 0
        self.local_plan_pub = rospy.Publisher("local_plan",Path,queue_size=1)
        rospy.spin()

    def OdomCB(self,data):
        self.rx = data.pose.pose.position.x
        self.ry = data.pose.pose.position.y

    def callback(self,data):
        print("subscribed")
                
        path_msg = Path()
        listener = tf.TransformListener()
        # yaw = 
        path_list = []
        path_msg.header.seq = rospy.Time.now()
        path_msg.header.frame_id = "base_link"

        # try:
        #     (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        #     (_, _, yaw) = euler_from_quaternion (rot)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     pass

        for pos in data.points:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = pos.x
            pose_msg.pose.position.y = pos.y
            path_list.append(pose_msg)
        path_msg.poses = path_list
        self.local_plan_pub.publish(path_msg)

if __name__ == "__main__":

    start = Path_test()
        