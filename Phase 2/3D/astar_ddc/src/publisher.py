#!/usr/bin/env python3

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist
import cv2

import numpy as np
import heapq
from Astar_ddc import Astar_DDC
from arena import Node, Robot, Graph
from utils import gazebo2map
# , save_data, load_data

import sys
from open_loop import open_loop_publisher
# from closed_loop import closed_loop_publisher
import os
if __name__ == '__main__':
    try:

        # converts the inputs range of (-5, 5) to range of (0,100)
        # START = [gazebo2map(float(sys.argv[1])), gazebo2map(float(sys.argv[2])), float(sys.argv[3])]
        # GOAL = [gazebo2map(float(sys.argv[4])), gazebo2map(float(sys.argv[5])), 0]

        START = [float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])]
        GOAL = [float(sys.argv[4]), float(sys.argv[5]), 0]
        clearance = float(sys.argv[6])
        rpm1, rpm2 = int(sys.argv[7]), int(sys.argv[8])

        print(START, GOAL, rpm1, rpm2, clearance)

        robot = Robot(rpm1, rpm2, clearance)
        planner = Astar_DDC(robot, START, GOAL, clearance)
        actions, waypoints, points = planner.search()

        if len(actions) == 0:
            print("Exiting... nothing to run here")
            exit()

        rospy.sleep(3.)

        open_loop_publisher(robot, (actions, waypoints))
        # closed_loop_publisher(robot, path)

    except rospy.ROSInterruptException:
        pass
