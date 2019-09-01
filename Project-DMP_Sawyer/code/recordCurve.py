#!/usr/bin/env python
import rospy
import intera_interface
from intera_interface import Gripper
import intera_external_devices
from geometry_msgs.msg import ( PoseStamped, Pose, Point, Quaternion, )
import math

def moveToNeutral(limb):
	angles = limb.joint_angles()

	# Manually assign joint angles to that of neutral position
	angles['right_j0']=0.0
	angles['right_j1']=-1.1799162119880
	angles['right_j2']=0.0
	angles['right_j3']=2.1800525671838624
	angles['right_j4']=0.0
	angles['right_j5']=0.5699964623054745
	angles['right_j6']=3.140002173611414

	limb.set_joint_position_speed(speed = 0.01)
	limb.move_to_joint_positions(angles)

def appendToFile(poseStr):
	with open('recordedPoints-Curve.txt', 'a') as f:
		f.write(poseStr)


def storeKeyframe(limb, is_gripping):
	current_position = limb.endpoint_pose()
	pose = ""
	pose = str(rospy.get_time())+","+str(current_position['position'].x)+","+str(current_position['position'].y) + ","+str(current_position['position'].z) +","+str(current_position['orientation'].x) +","+str(current_position['orientation'].y)+","+str(current_position['orientation'].z)+","+str(current_position['orientation'].w)
	pose = pose + "," +str(is_gripping) +"\n"
	appendToFile(pose)



def process():
	#Location pair 1
	'''start = [-0.4, 0.5, 0.3]
	goal = [0.3, 0.5, 0.06]'''	
	#Location pair 2 - Block to bowl
	'''start = [0.0, 0.5, 0.06]
	goal = [0.3, 0.5, 0.06]	'''
	#Location pair 3 - Move to block 
	start = [-0.4, 0.5, 0.06]
	goal = [0.3, 0.5, 0.06]

	#Duration of trajectory
	T = 2
	dt = 0.01

	overhead_orientation = Quaternion(x=-0.0249590815779,y=0.999649402929,z=0.00737916180073,w=0.00486450832011)

	rospy.init_node('testRecord1')

	limb = intera_interface.Limb('right')
	right_gripper = Gripper('right_gripper')

	moveToNeutral(limb)
	right_gripper.open()

	pose = Pose(position=Point(x=start[0], y=start[1], z=start[2]),orientation=overhead_orientation)
	#move to start position
	try:
		angles = limb.ik_request(pose, 'right_hand')
	    	limb.move_to_joint_positions(angles)
	except:
		print("erron in conversion or jointangles")

	rospy.sleep(1)

	dx = (goal[0] - start[0])/(T/dt)
	dy = (goal[1] - start[1])/(T/dt)
	dz = (goal[2] - start[2])/(T/dt)
	x = start[0]
	y = start[1]
	z = start[2]
	timeElapsed = 0.0
	while timeElapsed <= T:
		#print("in loop", dx, " ", dy, " ", dz)
		x += dx
		y += dy
		#z += dz
		#z = 0.06 + 0.2 * math.sin(timeElapsed/(T) * math.pi)
		dz = 0.1 * math.sin(timeElapsed/(T) * math.pi)
		pose = Pose(position=Point(x, y, z+dz),orientation=overhead_orientation)
		try:
			angles = limb.ik_request(pose, 'right_hand')
		    	limb.move_to_joint_positions(angles)
			storeKeyframe(limb, right_gripper.is_gripping())
		except:
			print("error in conversion or jointangles")
		timeElapsed += dt
		rospy.sleep(dt)
	rospy.sleep(1)


if __name__ == '__main__':
    try:
        process()
    except rospy.ROSInterruptException:
        pass#!/usr/bin/env python

