#!/usr/bin/env python

# --------Include modules---------------
from cmath import inf
from copy import copy
from distutils.log import info
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf
from numpy import array, vstack, delete
from functions import gridValue, informationGain, obstacleGain
from sklearn.cluster import MeanShift
from turtlebot3_aslam.msg import PointArray

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
globalmap = OccupancyGrid()
localmap = OccupancyGrid()
frontiers = []


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x


def mapCallBack(data):
    global mapData
    mapData = data


def globalMap(data):
    global globalmap
    globalmap = data


def localMap(data):
    global localmap
    localmap = data

# Node ----------------------------------------------
def node():
    global frontiers, mapData, global1, global2, global3, globalmap, localmap
    rospy.init_node('filter', anonymous=False)

    # Fetching all parameters
    map_topic = rospy.get_param('~map_topic', 'map')
    door_length = rospy.get_param('~door_length', 0.5)
    wall_thickness = rospy.get_param('~wall_thickness', 0.2)
    robot_length = rospy.get_param('~robot_length', 0.1)
    info_radius = rospy.get_param('~info_radius', 3.0)  # This can be smaller than the laser scanner range, >> smaller >> less computation time >> too small is not good, info gain won't be accurate
    threshold = rospy.get_param('~costmap_clearing_threshold', 30)
    goals_topic = rospy.get_param('~goals_topic', '/detected_points')
    rateHz = rospy.get_param('~rate', 100)
    global_costmap_topic = rospy.get_param(
        '~global_costmap_topic', '/move_base/global_costmap/costmap')
    local_costmap_topic = rospy.get_param(
        '~local_costmap_topic', '/move_base/local_costmap/costmap')
    robot_frame = rospy.get_param('~robot_frame', 'base_link')

    rate = rospy.Rate(rateHz)
    # -------------------------------------------
    # Subscribers
    rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
    rospy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)
    rospy.Subscriber(local_costmap_topic, OccupancyGrid, localMap)

    # Wait if map is not received yet
    while (len(mapData.data) < 1):
        rospy.loginfo('Waiting for the map')
        rospy.sleep(0.1)
        pass

    # Wait if any of robots' global costmap map is not received yet
    while (len(globalmap.data) < 1):
        rospy.loginfo('Waiting for the global costmap')
        rospy.sleep(0.1)
        pass

    global_frame = "/" + mapData.header.frame_id

    tfLisn = tf.TransformListener()
    tfLisn.waitForTransform(global_frame[1:], '/' + robot_frame, rospy.Time(0), rospy.Duration(10.0))

    rospy.Subscriber(goals_topic, PointStamped, callback=callBack,
                     callback_args=[tfLisn, global_frame[1:]])
    pub = rospy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rospy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rospy.Publisher('filtered_points', PointArray, queue_size=10)

    rospy.loginfo("the map and global costmaps are received")

    # Wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
    
    # Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rospy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

    # Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    points.lifetime = rospy.Duration()

    p = Point()

    p.z = 0

    pp = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rospy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

    # Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = rospy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rospy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
    # -------------------------------------------------------------------------
    # ---------------------     Main   Loop     -------------------------------
    # -------------------------------------------------------------------------
    while not rospy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # If there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front
        frontiers = copy(centroids)
        
        # -------------------------------------------------------------------------
        # Clearing old frontiers
        if len(centroids) > 1:
            z = 0
            while z < len(centroids):
                cond = False
                temppoint.point.x = centroids[z][0]
                temppoint.point.y = centroids[z][1]
                
                transformedPoint = tfLisn.transformPoint(globalmap.header.frame_id, temppoint)
                x = array([transformedPoint.point.x, transformedPoint.point.y])
                cond = gridValue(mapData, x) > threshold
                obsCond = obstacleGain(mapData, [centroids[z][0], centroids[z][1]], door_length / 2, threshold) > ((wall_thickness / 2) ** 2)
                infCond = informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius) == 0.0
                if cond or obsCond or infCond:
                    centroids = delete(centroids, (z), axis=0)
                    z = z-1
                z += 1
        
        # -------------------------------------------------------------------------
        # Publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filterpub.publish(arraypoints)
        pp = []
        for q in range(0, len(frontiers)):
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            pp.append(copy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        pub.publish(points)
        pub2.publish(points_clust)
        rate.sleep()
        
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
