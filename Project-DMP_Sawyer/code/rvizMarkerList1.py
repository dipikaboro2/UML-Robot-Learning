#!/usr/bin/env python
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

_filename = "reproducedSineMidway.txt"
#_filename = "recordedPoints-Curve.txt"
topic = 'visualization_marker_array'
publisher = rospy.Publisher(topic, MarkerArray, queue_size = 10)

rospy.init_node('drawTrajectory')

markerArray1 = MarkerArray()

count = 0
numLines = sum(1 for line in open(_filename))

#read file and add marker for every datapoint
with open(_filename) as f:
    for line in f:
	row = line.strip('\n').split(',')
	x = float(row[1])
	y = float(row[2])
	z = float(row[3])
	marker = Marker()
	marker.header.frame_id = "/base"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = 1.0
	marker.color.r = (1.0/numLines)*count
	marker.color.g = 1.0 -((1.0/numLines)*count)
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = x
	marker.pose.position.y = y 
	marker.pose.position.z = z 


	markerArray1.markers.append(marker)

	# Renumber the marker IDs
	id = 0
	for m in markerArray1.markers:
		m.id = id
		id += 1

	# Publish the MarkerArray
	publisher.publish(markerArray1)

	count += 1

	rospy.sleep(0.01)
	
	
	
count = 0
numLines = sum(1 for line in open(_filename))

#read file and add marker for every datapoint
with open('recordedPoints-Curve.txt') as f:
    for line in f:
	row = line.strip('\n').split(',')
	x = float(row[1])
	y = float(row[2])
	z = float(row[3])
	marker = Marker()
	marker.header.frame_id = "/base"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.05
	marker.scale.y = 0.05
	marker.scale.z = 0.05
	marker.color.a = 1.0
	marker.color.r = (1.0/numLines)*count
	marker.color.g = 1.0 -((1.0/numLines)*count)
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0
	marker.pose.position.x = x
	marker.pose.position.y = y 
	marker.pose.position.z = z 


	markerArray1.markers.append(marker)

	# Renumber the marker IDs
	id = len(markerArray1.markers)
	for m in markerArray1.markers:
		m.id = id
		id += 1

	# Publish the MarkerArray
	publisher.publish(markerArray1)

	count += 1

	rospy.sleep(0.01)

