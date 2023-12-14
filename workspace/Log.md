# Notes and issue

## 2023/11/19
### Notes 
* Only need to tuck once, at initial state, get all the block init pos. -> save time
* Pick up and drop can be in the same function, no need to seperate
* Given the init and target coordinate matrix (n*3), can move continuously between these coordinates in order. 
### Issues
* How to control the open extent of gripper
* How to personally choose the ik planning path each time (imperfect method: divide the path into several parts, each part define the end point)
* How to make the robot move faster
* How to reconginze the blocks and obtain their coordinates in base frame(how to add ar_track_alvar package to the workspace? ask TA)
* How to add control in it

## 2023/11/27
### Notes
* DONE: Successfully detect the position of two blocks and grab them to target position.
* To Do 1: Get the block target position from ursina, successfully place the block at the target.
* To Do 2: Fine tune the height 
* To Do 3: Grip the bloc based on the orientation in yaw. From AR_tag, get the yaw angle => angle of the end effector. So that the grip can grab blocks in different orientation.
* To Do 4: Detect AR tag (a piece of paper) as reference target position. 

## 2023/11/28
### Note
* Finish the baseline: 
	1.design the structure with ursina, convert the local coord to base frame coord 
	2.get the init pos of two blocks by vision
	3.get the target pos by calling the get_block_coordinates
	4.tuck the sawyer's init pos so that recognize the ar tag of two blocks
	5.use gripper and ik to grip and move the blocks from init to target

### Issues:
* Debug: mismatch of coordinates when y <0 in design process
* Try large scale blocks building
* Change the orientation of gripper so that blocks align in y axis 
* Add control algorithm into it

## 2023/11/29
### Note
* Update: The gripper can rotate so that the fingures can perfectly contact with the surface of the blocks. Making the pick-up behavior more stable.

### Issues:
* Same as 11/28

## 2023/12/2
### Issues:
* save the coordinate in json file according to the clip order rather than the distance order from origin
* coordinate of axis should be fixed
* AR tag recognition accuracy low
* large scale recognition
* active preception
* control stuff need to be added


### Notes:
* gripper can change its orientation when dropping so that it will not intercept with blocks in place. Achieved by getting the nearest block of the current placing block, get pos vector, get the axis that has largest number, if y axis, rotate 90 degree, other wise, not rotate.
* Should use lab7 circular trajectory to actively percept the ar tags. Not only solve the low accuracy problem, but add control stuff, moreover could do large scale task.

## 2023/12/3
### Issues:
* Test recognition accuracy (large scale)
* look up tag fine tuning
* Test essemble accuracy (large scale)
* Design good structure , demo
* Video recording
* PPT 
