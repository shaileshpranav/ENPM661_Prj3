#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import csv
import tf

import numpy as np
import sys


def pathMessages(waypoints):

    path = Path()
    path.header.frame_id = "/map"
    
    for point in waypoints:

        pose = PoseStamped()
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = 0

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        
        path.poses.append(pose)

    return path
 


def velocities(robot, action):
    r, L, dt = robot.radius, robot.wheelDistance, robot.dt 
    wL = float(action[0])
    wR = float(action[1])
    v = (r/2)* (wL + wR )
    w = (r/L) * (wL - wR )
    return w, v

def velocityMessages(robot, action):
    vel = Twist()
    lin_vel, ang_vel = velocities(robot, action)

    vel.linear.x = lin_vel
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = ang_vel

    return vel


def closestPoint(_pose, waypoints):
    current_point = np.array([_pose.position.x, _pose.position.y]).reshape(-1,2)
    waypoints = np.array(waypoints, dtype = float).reshape(-1, 2)

    difference = waypoints - current_point
    ssd = np.sum(np.square(difference), axis = 1) 
    idx = np.argmin(ssd)
    min_distance = np.sqrt(ssd[idx])
    delx, dely = difference[idx, 0], difference[idx, 1]

    min_distance = min_distance if (delx < 0) else -min_distance
    dx, dy = waypoints[idx, 0], waypoints[idx, 1] 
    
    return dx, dy, min_distance 

def poseMessage(trans, rot):
    _pose = Pose()
    _pose.position.x = trans[0]
    _pose.position.y = trans[1]
    _pose.position.z = trans[2]

    _pose.orientation.x = rot[0]
    _pose.orientation.y = rot[1]
    _pose.orientation.z = rot[2]
    _pose.orientation.w = rot[3]

    return _pose
    
def updateAngVel(vel_old, d, k  = 1.7):
    vel_new = vel_old    
    vel_new.angular.z = vel_old.angular.z + k * d
    return vel_new

def open_loop_publisher(robot, path_info):
    actions, waypoints = path_info
    rospy.sleep(3.)
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('publisher', anonymous=True)

    listener = tf.TransformListener()

    rospy.sleep(3.)

    path = pathMessages(waypoints)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = poseMessage(trans, rot)
            _, _, distance = closestPoint(current_pose, waypoints)
            

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("transformation unavailable between map and baselink ...")
            rospy.sleep(1.)
            continue

        action = [0,0] if (len(actions) == 0) else actions.pop(0)
        vel = velocityMessages(robot, action)

        #update omega
        if (abs(distance) > 0.1 and len(actions) > 0):
           vel =  updateAngVel(vel, distance)

        path_publisher.publish(path)
        vel_publisher.publish(vel)

        rate.sleep()
