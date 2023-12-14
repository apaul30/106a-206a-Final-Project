#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from intera_interface import gripper as robot_gripper
from get_coordinates import get_block_coordinates
import sys
import tf2_ros
import tf
import math

from trac_ik_python.trac_ik import IK
import intera_interface
from sawyer_pykdl import sawyer_kinematics
from paths.path_planner import PathPlanner
from paths.trajectories import LinearTrajectory, CircularTrajectory
from paths.paths import MotionPath
from controllers.controllers import ( 
    PIDJointVelocityController, 
    FeedforwardJointVelocityController
)
# def go_to(request):
    # # Send the request to the service
    # response = compute_ik(request)
    
    # # Print the response HERE
    # # print(response)
    # group = MoveGroupCommander("right_arm")

    # # Setting position and orientation target
    # group.set_pose_target(request.ik_request.pose_stamped)

    # # TRY THIS
    # # Setting just the position without specifying the orientation
    # ###group.set_position_target([0.5, 0.5, 0.0])

    # # Plan IK
    # plan = group.plan()
    # print(plan)
    # # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    # user_input = 'y'
    
    # # Execute IK if safe
    # if user_input == 'y':
    #     group.execute(plan[1])  
def active_perception(search_pos):
    # rospy.init_node('moveit_node')
    ik_solver = IK("base", "right_gripper_tip") # "right_hand_camera"
    limb = intera_interface.Limb("right")
    kin = sawyer_kinematics("right")
    tag_pos = search_pos #np.array([0.9, 0.198+0.1, 0.16])
    robot_trajectory = get_trajectory(limb, kin, ik_solver, tag_pos)

    planner = PathPlanner('right_arm')

    
    controller = get_controller('pid', limb, kin)
    # Move to the trajectory start position
    plan = planner.plan_to_joint_pos(robot_trajectory.joint_trajectory.points[0].positions)
    
    plan = planner.retime_trajectory(plan, 0.3)
    planner.execute_plan(plan[1])
    try:
        pass
        # input('Press <Enter> to execute the trajectory using YOUR OWN controller')
    except KeyboardInterrupt:
        sys.exit()
    # execute the path using your own controller.
    done = controller.execute_path(
        robot_trajectory, 
        rate=200, 
    )
    if not done:
        print('Failed to move to position')
        sys.exit(0)

def get_trajectory(limb, kin, ik_solver, tag_pos):
    """
    Returns an appropriate robot trajectory for the specified task.  You should 
    be implementing the path functions in paths.py and call them here
    
    Parameters
    ----------
    task : string
        name of the task.  Options: line, circle, square
    tag_pos : 3x' :obj:`numpy.ndarray`
        
    Returns
    -------
    :obj:`moveit_msgs.msg.RobotTrajectory`
    """
    num_way = 50
    task = "line" #"circle"

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform('base', 'right_hand', rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)

    current_position = np.array([getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')])
    print("Current Position:", current_position)

    if task == 'line':
        target_pos = tag_pos
        # target_pos[2] += 0.4 #linear path moves to a Z position above AR Tag.
        print("TARGET POSITION:", target_pos)
        trajectory = LinearTrajectory(start_position=current_position, goal_position=target_pos, total_time=9)
    elif task == 'circle':
        target_pos = tag_pos
        # breakpoint()
        # target_pos[2] += 0.5
        print("TARGET POSITION:", target_pos)
        trajectory = CircularTrajectory(center_position=target_pos, radius=0.1, total_time=20)

    else:
        raise ValueError('task {} not recognized'.format(task))
    
    path = MotionPath(limb, kin, ik_solver, trajectory)
    return path.to_robot_trajectory(num_way, True)

def get_controller(controller_name, limb, kin):
    """
    Gets the correct controller from controllers.py

    Parameters
    ----------
    controller_name : string

    Returns
    -------
    :obj:`Controller`
    """
    if controller_name == 'open_loop':
        controller = FeedforwardJointVelocityController(limb, kin)
    elif controller_name == 'pid':
        Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
        Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
        Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
        Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])
        # Kw = np.array([0., 0., 0., 0., 0., 0., 0.])

        controller = PIDJointVelocityController(limb, kin, Kp, Ki, Kd, Kw)
    else:
        raise ValueError('Controller {} not recognized'.format(controller_name))
    return controller



def rotate_gripper(i, raw_block_target_coord):
    if i >0:
        # breakpoint()
        placed_blocks = raw_block_target_coord[:i]
        current_block = raw_block_target_coord[i]

        min_idx = np.argmin(np.linalg.norm(placed_blocks - current_block, axis=1), axis=0)
        min_block = raw_block_target_coord[min_idx]


        min_dist = np.abs(current_block - min_block)
        # breakpoint()
        if np.argmax(min_dist) == 0: # contact along the x-axis
            gripper_orient = [0,1,0,0]
        elif np.argmax(min_dist) == 1:
            gripper_orient = [-np.sqrt(1/2), np.sqrt(1/2), 0, 0]
        else:
            gripper_orient = [0,1,0,0]
        
        return gripper_orient
    else:
        # breakpoint()
        gripper_orient = [0,1,0,0]
        return gripper_orient

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.

    :param roll: Roll angle (rotation around X-axis) in radians.
    :param pitch: Pitch angle (rotation around Y-axis) in radians.
    :param yaw: Yaw angle (rotation around Z-axis) in radians.
    :return: Quaternion [x, y, z, w]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr

    return np.array([qx, qy, qz, qw])

def pick_it_up(i, block_init_pos,block_init_orient, request, compute_ik,right_gripper):

    request.ik_request.pose_stamped.pose.position.x = block_init_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_init_pos[i,1]
    request.ik_request.pose_stamped.pose.position.z = block_init_pos[i,2] +0.3      
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'
    
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])    
    ################################################

    # Set the desired position of the end-effector (relative to the base frame)
    request.ik_request.pose_stamped.pose.position.x = block_init_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_init_pos[i,1]
    request.ik_request.pose_stamped.pose.position.z = block_init_pos[i,2]        
    request.ik_request.pose_stamped.pose.orientation.x = block_init_orient[i,0]
    request.ik_request.pose_stamped.pose.orientation.y = block_init_orient[i,1]
    request.ik_request.pose_stamped.pose.orientation.z = block_init_orient[i,2]
    request.ik_request.pose_stamped.pose.orientation.w = block_init_orient[i,3]
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'

    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])
        print('Closing')
        right_gripper.close()
        rospy.sleep(1.0)
    ################################################

    request.ik_request.pose_stamped.pose.position.x = block_init_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_init_pos[i,1]
    request.ik_request.pose_stamped.pose.position.z = block_init_pos[i,2] +0.3      
    request.ik_request.pose_stamped.pose.orientation.x = 0.0
    request.ik_request.pose_stamped.pose.orientation.y = 1.0
    request.ik_request.pose_stamped.pose.orientation.z = 0.0
    request.ik_request.pose_stamped.pose.orientation.w = 0.0
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'   
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])    
    return

def drop_it_down(i, block_target_pos, request, compute_ik,right_gripper, raw_block_target_coord):
    request.ik_request.pose_stamped.pose.position.x = block_target_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_target_pos[i,1] 
    request.ik_request.pose_stamped.pose.position.z = block_target_pos[i,2] + 0.3     
    request.ik_request.pose_stamped.pose.orientation.x = rotate_gripper(i, raw_block_target_coord)[0]
    request.ik_request.pose_stamped.pose.orientation.y = rotate_gripper(i, raw_block_target_coord)[1]
    request.ik_request.pose_stamped.pose.orientation.z = rotate_gripper(i, raw_block_target_coord)[2]
    request.ik_request.pose_stamped.pose.orientation.w = rotate_gripper(i, raw_block_target_coord)[3]
    # Send the request to the service
    response = compute_ik(request)

    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'
    
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])

    ################################################
    request.ik_request.pose_stamped.pose.position.x = block_target_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_target_pos[i,1]
    request.ik_request.pose_stamped.pose.position.z = block_target_pos[i,2]      
    request.ik_request.pose_stamped.pose.orientation.x = rotate_gripper(i, raw_block_target_coord)[0]
    request.ik_request.pose_stamped.pose.orientation.y = rotate_gripper(i, raw_block_target_coord)[1]
    request.ik_request.pose_stamped.pose.orientation.z = rotate_gripper(i, raw_block_target_coord)[2]
    request.ik_request.pose_stamped.pose.orientation.w = rotate_gripper(i, raw_block_target_coord)[3]
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'
    
    
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
    ################################################

    request.ik_request.pose_stamped.pose.position.x = block_target_pos[i,0]
    request.ik_request.pose_stamped.pose.position.y = block_target_pos[i,1] 
    request.ik_request.pose_stamped.pose.position.z = block_target_pos[i,2] + 0.3     
    request.ik_request.pose_stamped.pose.orientation.x = rotate_gripper(i, raw_block_target_coord)[0]
    request.ik_request.pose_stamped.pose.orientation.y = rotate_gripper(i, raw_block_target_coord)[1]
    request.ik_request.pose_stamped.pose.orientation.z = rotate_gripper(i, raw_block_target_coord)[2]
    request.ik_request.pose_stamped.pose.orientation.w = rotate_gripper(i, raw_block_target_coord)[3]
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    ###group.set_position_target([0.5, 0.5, 0.0])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'
 
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])
        
    return 

def tuck( request, compute_ik,right_gripper):
    # Set the desired position of the end-effector (relative to the base frame)
    request.ik_request.pose_stamped.pose.position.x = 0.9
    request.ik_request.pose_stamped.pose.position.y = 0.198
    request.ik_request.pose_stamped.pose.position.z = 0.16        
    request.ik_request.pose_stamped.pose.orientation.x = 0
    request.ik_request.pose_stamped.pose.orientation.y = np.sqrt(1/2)
    request.ik_request.pose_stamped.pose.orientation.z = 0
    request.ik_request.pose_stamped.pose.orientation.w = np.sqrt(1/2)
    # Send the request to the service
    response = compute_ik(request)
    
    # Print the response HERE
    # print(response)
    group = MoveGroupCommander("right_arm")

    # Setting position and orientation target
    group.set_pose_target(request.ik_request.pose_stamped)

    # TRY THIS
    # Setting just the position without specifying the orientation
    #group.set_position_target([0.888, 0.198, -0.021])

    # Plan IK
    plan = group.plan()
    print(plan)
    # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
    user_input = 'y'
    
    
    # Execute IK if safe
    if user_input == 'y':
        group.execute(plan[1])
        
        
          
    return

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.  

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
  

    r = rospy.Rate(10) # 10hz

    
    # TODO: initialize a tf buffer and listener as in lab 3

    try:
        # TODO: lookup the transform and save it in trans
        # The rospy.Time(0) is the latest available 
        # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
        target_name = "ar_marker_"+str(tag_number)
        trans = tfBuffer.lookup_transform("base",target_name , rospy.Time(0), rospy.Duration(10.0))
    except Exception as e:
        print(e)
        return True

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    tag_orient = [getattr(trans.transform.rotation, dim) for dim in ('x', 'y', 'z', 'w')]
    # breakpoint()
    return np.array(tag_pos), np.array(tag_orient)

def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    right_gripper = robot_gripper.Gripper('right_gripper')
    pick_up = True
    block_target_pos_central = np.array([0.7, -0.15, -0.15])
    raw_block_target_coord = get_block_coordinates(origin=block_target_pos_central)
    block_count = len(raw_block_target_coord)
    # breakpoint()
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    request.ik_request.pose_stamped.header.frame_id = "base"
    tuck(request, compute_ik,right_gripper)
    # breakpoint()
    # block_init_pos = np.array([[0.5, 0.5, -0.08],
    #                             [0.671, 0.472, -0.08]]) # 2*3 array, 2 blocks, 3D coordinates
    # block_target_pos = np.array([[0.5, -0.15, -0.08],
    #                             [0.50, -0.15, 0.045]]) # 2*3 array, 2 blocks, 3D coordinates    
    block_init_pos = np.zeros((block_count,3))
    
    block_init_orient = np.zeros((block_count,4))

    block_desired_init_orient = np.zeros((block_count,4))

    block_target_pos = np.zeros((block_count,3)) # init the init and target pos matrix
    # breakpoint()
    active_idx = []
    for i in range(block_count):
        init_orient = np.array([0,1,0,0])
        # breakpoint()
        # if type(lookup_tag(i)) == bool:


        if type(lookup_tag(i)) == bool:
            active_idx.append(i)
            # print("Launch Active Perception Process!")
            # search_pos = np.array([0.9, 0.161, 0.16])
            # active_perception(search_pos)
            # block_init_pos[i,:], block_init_orient[i,:]  = lookup_tag(i)
            # tuck(request, compute_ik,right_gripper)
        # breakpoint()
        else:
            block_init_pos[i,:], block_init_orient[i,:]  = lookup_tag(i)
        
            print("ar_tag_"+str(i)+" recognized")
        
        euler_ang = tf.transformations.euler_from_quaternion(block_init_orient[i])
        init_euler_ang = tf.transformations.euler_from_quaternion(init_orient)
        desired_init_euler_ang = np.array([init_euler_ang[0], init_euler_ang[1], init_euler_ang[2] + euler_ang[2]])
        # desired_init_euler_ang = np.array([init_euler_ang[0], init_euler_ang[1], init_euler_ang[2]])
        # breakpoint()
        block_desired_init_orient[i,:] = quaternion_from_euler(desired_init_euler_ang[0], desired_init_euler_ang[1],desired_init_euler_ang[2])
        # breakpoint()
        block_init_pos[i,2] += 0.02 # m Height Offset 

    if len(active_idx) != 0:
        print("Launch Active Perception Process!")
        search_pos = np.array([0.85, 0.161, 0.16])
        active_perception(search_pos)

    for i in active_idx:
        block_init_pos[i,:], block_init_orient[i,:]  = lookup_tag(i)
        print("ar_tag_"+str(i)+" recognized")
        
        euler_ang = tf.transformations.euler_from_quaternion(block_init_orient[i])
        init_euler_ang = tf.transformations.euler_from_quaternion(init_orient)
        desired_init_euler_ang = np.array([init_euler_ang[0], init_euler_ang[1], init_euler_ang[2] + euler_ang[2]])
        # desired_init_euler_ang = np.array([init_euler_ang[0], init_euler_ang[1], init_euler_ang[2]])
        # breakpoint()
        block_desired_init_orient[i,:] = quaternion_from_euler(desired_init_euler_ang[0], desired_init_euler_ang[1],desired_init_euler_ang[2])
        # breakpoint()
        block_init_pos[i,2] += 0.02 # m Height Offset 
        


    tuck(request, compute_ik,right_gripper)
    # block_target_pos_central = lookup_tag(13)
    
    for i in range(len(block_target_pos)):
        block_target_pos[i,:] = raw_block_target_coord[i]
    block_target_pos[i,2] += 0.02

    # breakpoint()                     
    # block_count = block_init_pos.shape[0]
    i = 0 # start from the first block
    while not rospy.is_shutdown() and i < block_count:
        # if not back:
        #     init_pos = input('Press [ Enter ]: ').split(' ')
        
        # Construct the request
        # request = GetPositionIKRequest()
        # request.ik_request.group_name = "right_arm"

        # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        # link = "right_gripper_tip"

        # request.ik_request.ik_link_name = link
        # request.ik_request.pose_stamped.header.frame_id = "base"
        # tuck(request, compute_ik,right_gripper)
        # breakpoint()source ~ee106a/sawyer_setup.bash
        try:
            if pick_up:
                pick_it_up(i, block_init_pos, block_desired_init_orient, request, compute_ik,right_gripper) 
                pick_up = False

            else:
                drop_it_down(i, block_target_pos, request, compute_ik, right_gripper, raw_block_target_coord)
                pick_up = True
                i += 1
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()


