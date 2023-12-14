#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped  # Import service type
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


def ik_client():
    # Initialize the client node
    rospy.init_node('ik_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/ExternalTools/right/PositionKinematicsNode/IKService')
    try:
        
        IK = rospy.ServiceProxy(
            '/ExternalTools/right/PositionKinematicsNode/IKService', SolvePositionIK)
        
        pos = input('Please input the target pos of glipper: \n').split(' ')

        request = SolvePositionIKRequest()
        MSG = PoseStamped()

        MSG.pose.position.x = float(pos[0])
        MSG.pose.position.y = float(pos[1])
        MSG.pose.position.z = float(pos[2])

        MSG.pose.orientation.x = 0.0
        MSG.pose.orientation.y = -1.0
        MSG.pose.orientation.z = 0.0
        MSG.pose.orientation.w = 0.0
        MSG.header.frame_id = "base"
        MSG.header.stamp = rospy.get_rostime()

        request.pose_stamp.append(MSG)
        request.tip_names = "right_hand"

       

        rospy.loginfo('Prompt the user to input (x,y,z)')
        

        # request.pose_stamp.append(pose)
        joint_angles = IK(request)
        print(joint_angles)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    ik_client()
