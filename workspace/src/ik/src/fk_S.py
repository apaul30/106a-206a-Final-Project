#!/usr/bin/env python
import numpy as np
import rospy
from intera_core_msgs.srv import SolvePositionFK,SolvePositionFKRequest




def fk_client():
    # Initialize the client node
    rospy.init_node('ik_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/ExternalTools/right/PositionKinematicsNode/FKService')
    try:
        
        FK = rospy.ServiceProxy(
            '/ExternalTools/right/PositionKinematicsNode/FKService', SolvePositionFK)
        
        joint_angles = input('Please input the target angle of each joint: \n').split(' ')


        request = SolvePositionFKRequest()
        request.configuration.name = ['right_j0','right_j1','right_j2','right_j3','right_j4','right_j5','right_j6']
        request.configuration.velocity = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        joint_ang = []
        for i in range(7):
            joint_ang.append(float(joint_angles[i]))

        request.configuration.position = joint_ang
        
        request.pose_stamp.append()
        MSG = PoseStamped()

        MSG.pose.position.x = float(pos[0])
        MSG.pose.position.y = float(pos[1])
        MSG.pose.position.z = float(pos[2])

        joint_pos = joint_angles


        rospy.loginfo('Prompt the user to input (x,y,z)')
        

        pos = FK(joint_pos)
        print(pos)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    fk_client()
