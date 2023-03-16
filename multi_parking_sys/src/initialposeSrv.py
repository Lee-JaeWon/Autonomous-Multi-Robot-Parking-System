#!/usr/bin/env python

import rospy
from cartographer_ros_msgs.srv import StartTrajectory, StartTrajectoryRequest

if __name__ == '__main__':
    try:
        rospy.init_node('start_trajectory_client')
        start_trajectory_service = rospy.ServiceProxy('/start_trajectory', StartTrajectory)
        request = StartTrajectoryRequest()

        # Set the parameters for the trajectory
        request.configuration_basename = rospy.get_param('~configuration_basename')
        request.urdf = rospy.get_param('~urdf')
        request.urdf_provided = True
        request.use_bag_transforms = False

        # Call the start_trajectory service
        response = start_trajectory_service(request)

        # Log the response
        if response.trajectory_id != '':
            rospy.loginfo("Started trajectory with ID {}".format(response.trajectory_id))
        else:
            rospy.logwarn("Failed to start trajectory")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
