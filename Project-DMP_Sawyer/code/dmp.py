#!/usr/bin/env python
import rospy
import intera_interface
from intera_interface import Gripper
import intera_external_devices
from geometry_msgs.msg import ( PoseStamped, Pose, Point, Quaternion, )
import numpy as np
import math

#_filename = "recordedPoints.txt"
#_filename = "recordedPoints-Line.txt"
_filename = "recordedPoints-Curve.txt"
_gripperZOffset = 0.15

#Constants
alpha = 3.99# 4.6
K = 2200#600#25
D = 93.8#50#10

timesteps = []  #time points, array size= number of recorded data points
S = []
X = []
F_tar = []
_Vel = []
_Acc = []
ReproducedTrajectory = []

T = 0.0   #Demo duration
start = []
goal = []

def _Write_S_F_tarFile(S, F_tar):
	i = 0
	with open('fs.txt', 'w') as f:
		while i < len(S):
			s = str(S[i])
			for x in F_tar[i]:
				s =s + "," + str(x)
			s += "\n"
			f.write(s)
			i += 1

def _Differentiate(dataArray, timesteps):
	_result = []
	i = 1  #row
	while i < (len(dataArray)):
		j = 0
		_v = []
		#len(dataArray[0])=7
		while j < len(dataArray[0]):   #iterate over each column, i.e. for each of x, y, z, o_x, o_y, ...
			_v.append((dataArray[i][j] - dataArray[i-1][j])/(timesteps[i] - timesteps[i-1]))
			j += 1
		_result.append(_v)
		i += 1
	return  _result

'''
Input: file name of trajectory file
	(File format: CSV- time, x, y, z, x_o, y_o, z_o, w_o, grip_state)
Function reads the file and initializes the "2D" array X and the timesteps list.
It also sets the start and goal variables.

Return: void'''
def _ReadTrajectoryFile(_filename):
	_tempTime = []	
	with open(_filename) as f:
	    _tempTime = [float(row.strip('\n').split(',')[0]) for row in f]
	#Set task duration variable
	global T
	#T = _tempTime[-1] - _tempTime[0]
	#print("T=", T)
	#_prevTime = _tempTime[0]
	for t in _tempTime:
		'''timesteps.append(t - _prevTime)   #difference between consecutive steps
		_prevTime = t'''
		timesteps.append(t - _tempTime[0])   #time values starting from 0.0
        T = timesteps[-1]
	#print(timesteps)
	#populating X values
	with open(_filename) as f:
	    for line in f:
		row = line.strip('\n').split(',')
		pose = []
		pose.append(float(row[1]))	#x
		pose.append(float(row[2]))	#y
		pose.append(float(row[3])) 	#z
		pose.append(float(row[4]))	#q_x
		pose.append(float(row[5]))	#q_y
		pose.append(float(row[6]))	#q_z
		pose.append(float(row[7]))	#q_w
		X.append(pose)

	#set start and goal pose variables
	for x in X[0]:
		start.append(x)
	for x in X[-1]:
		goal.append(x)

def _EvaluateSvalues(timesteps):
	#T = timesteps[-1]???
	for t in timesteps:
	    s = math.exp(((-alpha)/T)*t)
	    S.append(s)


#This is the "Learn function"
def Compute_F_tar(_Vel, _Acc):
	i = 0
	while i < len(S):
		j = 0
		_ft = []
		
		while j < len(X[0]):
			#g1 = (D * _Vel[i][j] + tau*_Acc[i][j])/K
			#g2 = (goal[j] - X[i][j])
			#g3 = (goal[j] - start[j])*S[i]
			#_ft.append((D * _Vel[i][j] + tau*_Acc[i][j])/K - (goal[j] - X[i][j]) + (goal[j] - start[j])*S[i])
			_ft.append((D * _Vel[i][j] + T*_Acc[i][j])/K - (goal[j] - X[i][j]) + (goal[j] - start[j])*S[i])
			j += 1
		F_tar.append(_ft)
		i += 1
	return F_tar
		
	

def dmp():
	global _Acc
	global _Vel
	global F_tar
	_ReadTrajectoryFile(_filename)
	_EvaluateSvalues(timesteps)
	
	#Computing vel, or X_dot list
	_Vel = _Differentiate(X, timesteps)
	#Insert initial velocity
	_tv = [x for x in _Vel[0]]
	_Vel.insert(0, _tv)
	
	#Computing acc, or X_dot_dot list
	_Acc = _Differentiate(_Vel, timesteps)
	#Insert initial acc
	_ta = [x for x in _Acc[0]]
	_Acc.insert(0, _ta)
	
	F_tar = Compute_F_tar(_Vel, _Acc)
	print("F_tar dimension: ", len(F_tar), " ,", len(F_tar[0]))
	
	_Write_S_F_tarFile(S, F_tar)

if __name__ == '__main__':
    try:
        dmp()
    except rospy.ROSInterruptException:
        pass
