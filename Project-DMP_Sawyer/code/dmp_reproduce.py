#!/usr/bin/env python
import rospy
import intera_interface
from intera_interface import Gripper
import intera_external_devices
from geometry_msgs.msg import ( PoseStamped, Pose, Point, Quaternion, )
import numpy as np
import math

rospy.init_node('dmp_rep')

limb = intera_interface.Limb('right')
right_gripper = Gripper('right_gripper')

_filename = "fs.txt"
_gripperZOffset = 0.15

#Constants
alpha = 3.99 #4.6
K = 2500.#600#9 #25
D = 93.8#50#6 #10


timesteps = []  #time points, array size= number of recorded data points
dt = 0.045
S = []
learnt_S_F = []
X = []
F = []
VelList = []
AccList = []
ReproducedTrajectory = []

T = 10.0   #Demo duration
#start = [-0.398695625642,0.501994422518,0.184333719791,-0.024962931192,0.999649680034,0.00730676641393,0.00489698262251]
start = [-0.398695625642,0.501994422518,-0.084333719791,-0.024962931192,0.999649680034,0.00730676641393,0.00489698262251]
#goal = [-0.00285264455428,0.500049139986,-0.0730808263207,-0.0269405692855,0.999599634644,0.00681364533193,0.00532450780372]
goal = [0.3, 0.519,-0.0730808263207,-0.0769405692855,0.999599634644,0.00681364533193,0.00532450780372]  #tested
#start = [-0.398699815268,0.501964379495,-0.0756673248489,-0.0249744112657,0.999649433124,0.00730444350896,0.00489231453681]
#goal = [0.297678880514,0.499779438493,-0.073709261062,-0.0257792757035,0.999643056009,0.00575198350564,0.00401300413119]

def _Read_S_F_tarFile():
	i = 0
	with open(_filename, 'r') as f:
		for line in f:
			row = line.strip('\n').split(',')
			learnt_S_F.append([float(i) for i in row])


def _EvaluateSvalues():
	global timesteps
	timesteps = [x*dt for x in range(0, int(T/dt))]
	
	for t in timesteps:
	    s = math.exp(((-alpha)/T)*t)
	    S.append(s)
	print("Len of S: ", len(S))


#Compute F values and store in list F
def _interpolate():
	'''reversed_old_F = learnt_S_F[:]
	reversed_old_F.reverse()'''
	global F
	old_S = []
	old_F = []
	tempF = []
	for x in learnt_S_F:
		old_S.append(x[0])
		old_F.append(x[1:])
	old_S.reverse()
	old_F.reverse()
	S.reverse()
	i = 0
	while i < len(old_F[0]): #7
		f = []
		for x in old_F:
			f.append(x[i])
		y = []
		y = np.interp(np.array(S), np.array(old_S), np.array(f)).tolist()
		tempF.append(y)
		i += 1
	
	F = list(map(list, zip(*tempF)))
	F.reverse()
	S.reverse()


def _ComputeTrajectory():
	global X
	global VelList
	global AccList
	X.append(start)
	VelList.append([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
	_interpolate()
	i = 1
	j = 0
	while i < len(S):
		_vel = []
		_x = []
		while j < 7:
			v_dot = (K * (goal[j] - X[i-1][j]) - D * VelList[i-1][j] - K * (goal[j] - start[j]) * S[i] + K * F[i][j])/T
			v = VelList[i-1][j]+dt*v_dot
			_vel.append(v)
			x_dot = v / T
			x = X[i-1][j]+dt*x_dot
			_x.append(x)
			j += 1
		VelList.append(_vel)
		X.append(_x)
		'''
		#computing timesteps
		timesteps.append(dt*i)
		'''
		j = 0
		i += 1
	print("X dim 2: ", len(X)," , ",len(X[2]))
	print(len(timesteps))
		
def _appendToFile(row):
	with open('reproducedPoints-Line-K2200.txt', 'a') as f:
		f.write(row)

def _recordFile():
	#i = 0
	i = 1
	print("X[2] len:", len(X[2]))
	while i < len(timesteps):
		x = X[i][0]
		y = X[i][1]
		z = X[i][2]
		o_x = X[i][3]/math.sqrt(X[i][3]**2+X[i][4]**2+X[i][5]**2+X[i][6]**2)
		o_y = X[i][4]/math.sqrt(X[i][3]**2+X[i][4]**2+X[i][5]**2+X[i][6]**2)
		o_z = X[i][5]/math.sqrt(X[i][3]**2+X[i][4]**2+X[i][5]**2+X[i][6]**2)
		o_w = X[i][6]/math.sqrt(X[i][3]**2+X[i][4]**2+X[i][5]**2+X[i][6]**2)
		row = str(timesteps[i])+","+str(X[i][0])+","+str(X[i][1])+","+str(X[i][2])+","+str(X[i][3])+","+str(X[i][4])+","+str(X[i][5])+","+str(X[i][6])+"\n"
		#_appendToFile(row)
		
		'''
		pose = Pose()
		pose.position.x = x
		pose.position.y = y
		pose.position.z = z + _gripperZOffset
		
		pose.orientation.x = o_x
		pose.orientation.y = o_y
		pose.orientation.z = o_z
		pose.orientation.w = o_w
		#pose.orientation.x = 0.0249590815779
		#pose.orientation.y = 0.999649402929
		#pose.orientation.z = 0.00737916180073
		#pose.orientation.w = 0.00486450832011
		#overhead_orientation = Quaternion(x=-0.0249590815779,y=0.999649402929,z=0.00737916180073,w=0.00486450832011)
		
		try:
			angles = limb.ik_request(pose, 'right_hand')
		    	limb.move_to_joint_positions(angles)
		    	rospy.sleep(timesteps[i] - timesteps[i-1])
		except:
			print("erron in conversion or jointangles")
		
		i += 1
		'''
		
		
	right_gripper.open()
	overhead_orientation = Quaternion(x=-0.0249590815779,y=0.999649402929,z=0.00737916180073,w=0.00486450832011)
	rospy.sleep(1)
	ht = 0.001
	while ht <= 0.1:#0.141:
		pose = Pose(position=Point(x=X[-1][0], y=X[-1][1], z=X[-1][2]+_gripperZOffset-ht),orientation=overhead_orientation)
		try:
			angles = limb.ik_request(pose, 'right_hand')
		    	limb.move_to_joint_positions(angles)
		except:
			print("erron in conversion or jointangles")
		ht = ht + 0.001
		rospy.sleep(0.01)
		#rospy.sleep(.1)
	rospy.sleep(1)
	right_gripper.close()
	rospy.sleep(0.5)

def reproduce():
	global T
	T = 9.07
	_Read_S_F_tarFile()
	_EvaluateSvalues()
	_ComputeTrajectory()
	_recordFile()

if __name__ == '__main__':
    try:
        reproduce()
    except rospy.ROSInterruptException:
        pass
