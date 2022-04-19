#!/usr/bin/env python

# --------Include modules---------------
from copy import copy
from tkinter import W
import rospy
import math
from nav_msgs.msg import OccupancyGrid
from turtlebot3_aslam.msg import PointArray
from numpy import array
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
    sigma = rospy.get_param('~sigma', 0.5)  # This is the standard deviation of the gaussian distribution
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

        gain_record = []
        revenue_record = []
        centroid_record = []

        for ip in range(0, len(centroids)):
            cost = norm(turtle.getPosition() - centroids[ip])
            information_gain = infoGain[ip]

            revenue = 1 / sigma * math.sqrt(2 * math.pi) * math.exp(-0.5 * ((cost - info_radius / 2) / sigma) ** 2)
            gain_record.append(float(information_gain))
            revenue_record.append(revenue)
            centroid_record.append(centroids[ip])

        rospy.loginfo("Number of frontiers: %d", len(gain_record))
        rospy.logdebug("Information Gain: " + str(gain_record))
        rospy.logdebug("Revenue record: " + str(revenue_record))
        rospy.logdebug("Centroid record: " + str(centroid_record))

        # -------------------------------------------------------------------------
        if len(revenue_record) > 0:
            winner_id = revenue_record.index(max(revenue_record))
            # if revenue_record[winner_id] - prevRevenue:
            rospy.loginfo("Assigning robot " + turtle.name + "to frontier " + str(centroid_record[winner_id]))
            rospy.loginfo("Revenue: " + str(revenue_record[winner_id]))
            rospy.loginfo("Information Gain: " + str(gain_record[winner_id]))
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
