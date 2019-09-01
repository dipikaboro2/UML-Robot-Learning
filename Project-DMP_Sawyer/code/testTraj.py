#!/usr/bin/env python
import rospy
import intera_interface
from intera_interface import Gripper
import intera_external_devices
from geometry_msgs.msg import ( PoseStamped, Pose, Point, Quaternion, )

import csv

_filename = "recordedPoints-Curve.txt"
#_filename = "recordedPoints-Line.txt"
#_filename = "reproducedPoints-Line-K2200.txt"

_useTime = 0
_gripperZOffset = 0.15

rospy.init_node('teleopReadMove')

limb = intera_interface.Limb('right')
right_gripper = Gripper('right_gripper')

#get initial time
with open(_filename) as f:
    for line in f:
	prevTime = rospy.Time.from_sec(float(line.strip('\n').split(',')[0]))


angles = limb.joint_angles()

# Manually assign joint angles to that of neutral position
angles['right_j0']=0.0
angles['right_j1']=-1.1799162119880
angles['right_j2']=0.0
angles['right_j3']=2.1800525671838624
angles['right_j4']=0.0
angles['right_j5']=0.5699964623054745
angles['right_j6']=3.140002173611414

limb.move_to_joint_positions(angles)
#limb.move_to_neutral()
angles = limb.joint_angles()

i = 1
with open(_filename) as f:
    for line in f:
	#print("reading rows")
	row = line.strip('\n').split(',')
	if _useTime == 1:
		timeNow = rospy.Time.from_sec(float(row[0]))
		rospy.sleep(timeNow - prevTime)
	
	pose = Pose()
	pose.position.x = float(row[1])
	pose.position.y = float(row[2])
	pose.position.z = float(row[3]) + _gripperZOffset
	'''pose.orientation.x = 0.0249590815779
	pose.orientation.y = 0.999649402929
	pose.orientation.z = 0.00737916180073
	pose.orientation.w = 0.00486450832011'''
	pose.orientation.x = float(row[4])
	pose.orientation.y = float(row[5])
	pose.orientation.z = float(row[6])
	pose.orientation.w = float(row[7])
	
	try:
		angles = limb.ik_request(pose, 'right_hand')
	    	limb.move_to_joint_positions(angles)
	    	print(i)
	    	rospy.sleep(.01)
		i += 1
	except:
		print("erron in conversion or jointangles")
		print(i)
		i += 1


	#print("Moving\n", angles)
	if _useTime == 1:
		prevTime = timeNow
