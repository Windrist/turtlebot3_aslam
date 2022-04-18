#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
import turtle
import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf
from turtlebot3_aslam.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from functions import robot, informationGain
from numpy.linalg import norm

# Subscribers's Callbacks ------------------------------
mapData = OccupancyGrid()
frontiers = []


def callBack(data):
	global frontiers
	frontiers = []
	for point in data.points:
		frontiers.append(array([point.x, point.y]))


def mapCallBack(data):
	global mapData
	mapData = data


# Node ----------------------------------------------
def node():
	global frontiers, mapData
	rospy.init_node('assigner', anonymous=False)

	# Fetching all parameters
	map_topic = rospy.get_param('~map_topic', 'map')
	info_radius = rospy.get_param('~info_radius',
								  1.0)  # This can be smaller than the laser scanner range, >> smaller >> less computation time >> too small is not good, info gain won't be accurate
	hysteresis_gain = rospy.get_param('~hysteresis_gain',
									  2.0)  # Bigger than 1 (biase robot to continue exploring current region
	info_multiplier = rospy.get_param('~info_multiplier', 3.0)
	frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
	delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
	rateHz = rospy.get_param('~rate', 100)

	rate = rospy.Rate(rateHz)
	# -------------------------------------------
	# Subscribers
	rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
	rospy.Subscriber(frontiers_topic, PointArray, callBack)
	# ---------------------------------------------------------------------------------------------------------------

	# Wait if map is not received yet
	while (len(mapData.data) < 1):
		pass
	turtle = robot('')
	turtle.sendGoal(turtle.getPosition())

	# Wait if no frontier is received yet
	while len(frontiers) < 1:
		pass
	centroids = copy(frontiers)
	# -------------------------------------------------------------------------
	# ---------------------     Main   Loop     -------------------------------
	# -------------------------------------------------------------------------
	prevRevenue = 0
	while not rospy.is_shutdown():
		centroids = copy(frontiers)

		# -------------------------------------------------------------------------
		# Get information gain for each frontier point
		infoGain = []
		for ip in range(0, len(centroids)):
			infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
		# -------------------------------------------------------------------------
		# Calculate the Information Gain of each frontier point
		revenue_record = []
		centroid_record = []

		for ip in range(0, len(centroids)):
			cost = norm(turtle.getPosition() - centroids[ip])
			information_gain = infoGain[ip]

			if information_gain > 1.0:
				revenue = prevRevenue + math.exp(-1 / hysteresis_gain * cost) + information_gain * math.exp(
					-1 / info_multiplier * cost)
				revenue_record.append(revenue)
				centroid_record.append(centroids[ip])

		rospy.loginfo("Revenue record: " + str(revenue_record))
		rospy.loginfo("Centroid record: " + str(centroid_record))

		# -------------------------------------------------------------------------
		if len(revenue_record) > 0:
			winner_id = revenue_record.index(max(revenue_record))
			if revenue_record[winner_id] >= prevRevenue:
				rospy.loginfo("Assigning robot " + turtle.name + "to frontier " + str(centroid_record[winner_id]))
				turtle.sendGoal(centroid_record[winner_id])
				prevRevenue = revenue_record[winner_id]
				rospy.sleep(delay_after_assignement)

		# -------------------------------------------------------------------------
		rate.sleep()
		# -------------------------------------------------------------------------

if __name__ == '__main__':
	try:
		node()
	except rospy.ROSInterruptException:
		pass