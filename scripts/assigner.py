#!/usr/bin/env python

# --------Include modules---------------
import rospy
import math
from copy import copy
from nav_msgs.msg import OccupancyGrid
from turtlebot3_aslam.msg import PointArray
from numpy import array, delete
from functions import robot, informationGain, informationProbability
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
    frontiers_topic = rospy.get_param('~frontiers_topic', '/filtered_points')
    info_radius = rospy.get_param('~info_radius', 1.0)
    info_multiplier = rospy.get_param('~info_multiplier', 3.0)		
    hysteresis_radius = rospy.get_param('~hysteresis_radius', 3.0)
    hysteresis_gain = rospy.get_param('~hysteresis_gain', 2.0)
    delay_after_assignement = rospy.get_param('~delay_after_assignement', 0.5)
    rateHz = rospy.get_param('~rate', 100)

    rate = rospy.Rate(rateHz)
    
    # -------------------------------------------
    # Subscribers
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(frontiers_topic, PointArray, callBack)

    # ---------------------------------------------------------------------------------------------------------------

    # Wait if map is not received yet
    while len(mapData.data) < 1:
        pass
    
    # Wait if no frontier is received yet
    while len(frontiers) < 1:
        pass
    centroids = copy(frontiers)
    turtle = robot('')
   
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        centroids = copy(frontiers)
        
        # -------------------------------------------------------------------------
        # Get information gain for each frontier point
        infoGain = []
        infoProb = []
        for ip in range(0, len(centroids)):
            infoGain.append(informationGain(mapData, [centroids[ip][0], centroids[ip][1]], info_radius))
            cond, prob = informationProbability(mapData, [centroids[ip][0], centroids[ip][1]], info_radius)
            infoProb.append(prob)
            
        # -------------------------------------------------------------------------
        # Calculate the Information Gain of each frontier point
        revenue_record = []
        centroid_record = []

        for ip in range(0, len(centroids)):
            cost = norm(turtle.getPosition() - centroids[ip])
            
            # information_gain = infoGain[ip]
            # if (norm(turtle.getPosition() - centroids[ip]) <= hysteresis_radius):
            #     information_gain *= hysteresis_gain
            
            # revenue = information_gain * info_multiplier - cost
            
            information_probability = infoGain[ip]
            revenue = information_probability * math.exp(-hysteresis_gain * cost)
               
            revenue_record.append(revenue)
            centroid_record.append(centroids[ip])
        
        rospy.loginfo("Number of frontiers: %d", len(revenue_record))
        rospy.logdebug("Revenue record: " + str(revenue_record))
        rospy.logdebug("Centroid record: " + str(centroid_record))

        # -------------------------------------------------------------------------
        if len(revenue_record) > 0:
            winner_id = revenue_record.index(max(revenue_record))
            rospy.loginfo("Assigning robot " + turtle.name + "to frontier " + str(centroid_record[winner_id]))
            rospy.loginfo("Revenue: " + str(revenue_record[winner_id]))
            turtle.sendGoal(centroid_record[winner_id])
            rospy.sleep(delay_after_assignement)
        # -------------------------------------------------------------------------
        rate.sleep()
    # -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
