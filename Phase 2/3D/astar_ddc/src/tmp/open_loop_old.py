
import rospy
import tf

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist

import numpy as np
import sys

class PathPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('path', Path, queue_size=10)

    def publish(self, path_msgs):
        self.pub.publish(path_msgs)
        rospy.sleep(1.0)

    def update_msg(self, path_points):
        path = Path()
        path.header.frame_id = "/map"    
        for point in path_points:
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


def closestPoint(_pose, path_points):
    current_point = np.array([_pose.position.x, _pose.position.y]).reshape(-1,2)
    path_points = np.array(path_points, dtype = float).reshape(-1, 2)

    difference = path_points - current_point
    ssd = np.sum(np.square(difference), axis = 1) 
    idx = np.argmin(ssd)
    min_dist = np.sqrt(ssd[idx])
    delx, dely = difference[idx, 0], difference[idx, 1]

    min_dist = min_dist if (delx < 0) else -min_dist
    dx, dy = path_points[idx, 0], path_points[idx, 1] 
    
    return dx, dy, min_dist 

def updateOmega(vel, d, k  = 1.7):
    new_vel = vel
    omega = vel.angular.z
    omega_new = omega + k * d
    new_vel.angular.z = omega_new
    return new_vel


class VelocityPublisher:

    def __init__(self, robot):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.robot = robot

    def velocities(self, action):
        r, L, dt = self.robot.radius, self.robot.wheelDistance, self.robot.dt 
        wL, wR = action
        v = (r/2)* (wL + wR )
        w = (r/L) * (wL - wR )
        return w, v
        
    def publish(self, action, min_dist=0, omega_update=False):
        vel = Twist()

        linvel, angvel = self.velocities(action)

        vel.linear.x = linvel
        vel.linear.y = 0
        vel.linear.z = 0

        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = angvel

        if (abs(min_dist) > 0.1 and omega_update):
            vel =  updateOmega(vel, min_dist)

        self.pub.publish(vel)
        # rospy.sleep(10.0)



def open_loop_publisher(robot, path):

    """
    takes the path of nodes from start to goal and calls the publishers to navigate robot . 
    """
    
    rospy.sleep(3.)

    vel_pub = VelocityPublisher(robot)
    path_pub = PathPublisher()
    rospy.init_node('publisher', anonymous=True)
    listener = tf.TransformListener()

    # velocity_list, pose_list = retrieve_commands(robot, path)
    actions, points = path

    pose_list_msgs = path_pub.update_msg(points)
     

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():                    
        try:

            (trans,rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            current_pose = poseMessage(trans, rot)
            
            dx, dy, min_dist = closestPoint(current_pose, points)
            #print(min_dist)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("No tf available between map and baselink......................")
            rospy.sleep(1.)
            continue

    
        move = [0,0] if len(actions) == 0 else actions.pop(0)
        msg_str = "Publishing vel"
        rospy.loginfo(msg_str)
        rospy.loginfo(move)

        path_pub.publish(pose_list_msgs)
        vel_pub.publish(move, min_dist, len(actions) > 0)
        
        rate.sleep()
    print("end")
