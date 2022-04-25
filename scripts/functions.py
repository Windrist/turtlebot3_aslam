import rospy
import tf
import math
import random
import numpy as np
from numpy import array, identity, floor, transpose
from numpy.linalg import inv, det, norm, multi_dot
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped


class robot:
    goal = MoveBaseGoal()
    start = PoseStamped()
    end = PoseStamped()

    def __init__(self, name):
        self.assigned_point = []
        self.name = name
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        self.plan_service = rospy.get_param('~plan_service', '/move_base/make_plan')
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0), rospy.Duration(10.0))
        cond = 0
        while cond == 0:
            try:
                rospy.loginfo('Waiting for the robot transform')
                (trans, rot) = self.listener.lookupTransform(self.global_frame, '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        self.assigned_point = self.position
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client.wait_for_server()
        robot.goal.target_pose.header.frame_id = "map"
        robot.goal.target_pose.header.stamp = rospy.Time.now()
        rospy.wait_for_service(self.plan_service)
        self.make_plan = rospy.ServiceProxy(self.name + self.plan_service, GetPlan)
        robot.start.header.frame_id = self.global_frame
        robot.end.header.frame_id = self.global_frame

    def getPosition(self):
        cond = 0
        while cond == 0:
            try:
                (trans, rot) = self.listener.lookupTransform(self.global_frame, self.name + '/' + self.robot_frame, rospy.Time(0))
                cond = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                cond = 0
        self.position = array([trans[0], trans[1]])
        return self.position

    def sendGoal(self, point):
        robot.goal.target_pose.pose.position.x = point[0]
        robot.goal.target_pose.pose.position.y = point[1]
        robot.goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(robot.goal)
        self.assigned_point = array(point)

    def cancelGoal(self):
        self.client.cancel_goal()
        self.assigned_point = self.getPosition()

    def getState(self):
        return self.client.get_state()

    def makePlan(self, start, end):
        robot.start.pose.position.x = start[0]
        robot.start.pose.position.y = start[1]
        robot.end.pose.position.x = end[0]
        robot.end.pose.position.y = end[1]
        start = self.listener.transformPose(self.name+'map', robot.start)
        end = self.listener.transformPose(self.name+'map', robot.end)
        plan = self.make_plan(start=start, goal=end, tolerance=0.0)
        return plan.plan.poses
# ________________________________________________________________________________


def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    index = int(floor((Xp[1] - Xstarty) / resolution) * width + floor((Xp[0] - Xstartx) / resolution))
    return index

def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + i / mapData.info.width * mapData.info.resolution
    x = mapData.info.origin.position.x + i % mapData.info.width * mapData.info.resolution
    return array([x, y])
# ________________________________________________________________________________


def informationGain(mapData, point, r):
    infoGain = 0.0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region  * (mapData.info.width + 1)
    end_index = index + r_region * (mapData.info.width + 1)
    
    for sample_index in range(init_index, end_index + 1):
        if sample_index >= 0 and sample_index < len(mapData.data):
            if (mapData.data[sample_index] == -1 and norm(array(point) - point_of_index(mapData, sample_index)) <= r):
                infoGain += 1
    return infoGain * (mapData.info.resolution ** 2)
# ________________________________________________________________________________


def obstacleProbability(mapData, point, r, threshold):
    obsProb = 0
    index = index_of_point(mapData, point)
    r_region = int(r / mapData.info.resolution)
    init_index = index - r_region  * (mapData.info.width + 1)
    end_index = index + r_region * (mapData.info.width + 1)
    sigma = 100 - threshold
    
    for sample_index in range(init_index, end_index + 1):
        if sample_index >= 0 and sample_index < len(mapData.data):
            obsGain = mapData.data[sample_index]
            prob = math.exp(-0.5 * (((obsGain - 100) / sigma) ** 2)) / math.sqrt(2 * math.pi)
            obsProb += prob
    return obsProb / (2 * r_region * (mapData.info.width + 1))
# ________________________________________________________________________________


def gridValue(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y

    width = mapData.info.width
    Data = mapData.data
    # returns grid value at "Xp" location
    # map data:  100 occupied      -1 unknown       0 free
    index = floor((Xp[1] - Xstarty) / resolution) * width + floor((Xp[0] - Xstartx) / resolution)

    if int(index) < len(Data):
        return Data[int(index)]
    else:
        return 100
