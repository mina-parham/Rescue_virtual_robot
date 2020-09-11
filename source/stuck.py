#!/usr/bin/env python
import rospy
import math
import threading
import numpy
import time
from nav_msgs.srv import GetPlan ,GetPlanRequest
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import Bool
import roslib
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
import tf
import random
global cm, cm_width, cm_height

def CallCostMap(costmap):
    global cm, cm_width, cm_height
    cm = costmap.data
    cm_width = costmap.info.width
    cm_height = costmap.info.height
    print("2")
def signf(num):
    if num >= 0 :
        return 1
    else :
        return -1

def Search_Stuck_Point():
    global cm_width, cm_height , SP, mylocalmap, CPoint, costmapsize
    global cm
    print("searching...")
    mylocalmap = cm
    Sx = cm_width/2
    Sy = cm_height/2
    costmapsize = cm_height
    start = Sy * costmapsize + Sx 
    see = [start]
    seen = []
    while True:
        if not see:
            break
        CPoint = see[0]
        del see[0]
        seen.append(CPoint)
        x = CPoint % costmapsize
        y = CPoint // costmapsize
        if mylocalmap[CPoint] == 100:
            SP = CPoint 
            print(mylocalmap)
            return SP
        if x-1 >= 0:
            if not ((y)*costmapsize + x-1 in see) and not ((y)*costmapsize + x-1 in seen):
                see.append((y)*costmapsize + x-1)

        if x+1 <= costmapsize:
            if not ((y)*costmapsize + x+1 in see) and not ((y)*costmapsize + x+1 in seen):
                see.append((y)*costmapsize + x+1)

        if y-1 >= 0:
            if not ((y-1)*costmapsize + x in see) and not ((y-1)*costmapsize + x in seen):
                see.append((y-1)*costmapsize + x)

        if y+1 <= costmapsize:
            if not ((y+1)*costmapsize + x in see) and not ((y+1)*costmapsize + x in seen):
                see.append((y+1)*costmapsize + x)
def cmd_vel(R):
    global r
    r=R
    robot_pub = rospy.Publisher("/sos_rs1/cmd_vel",Twist, queue_size=1)
    twist = Twist()
    twist2 = Twist()
    twist3 = Twist()
    twist.linear.x = 0.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    twist2.linear.x= 0.0
    twist2.linear.y = 0.0
    twist2.linear.z = 0.0
    twist2.angular.x = 0.0
    twist2.angular.y = 0.0
    twist2.angular.z = 0.0
    twist3.linear.x= 0.0#
    twist3.linear.y = 0.0
    twist3.linear.z = 0.0
    twist3.angular.x = 0.0
    twist3.angular.y = 0.0
    twist3.angular.z = 9*R
    end_time = time.time() + 1
    while time.time() < end_time:#
        robot_pub.publish(twist3)
    rospy.Rate(1).sleep
    end_time = time.time() + 1
    while time.time() < end_time:
        robot_pub.publish(twist2)
    rospy.Rate(1).sleep
    rospy.Rate(1).sleep
    rospy.Rate(1).sleep
    rospy.Rate(1).sleep
    rospy.Rate(1).sleep
    end_time = time.time() + 1
    while time.time() < end_time:
        robot_pub.publish(twist)
    rospy.Rate(1).sleep
    end_time = time.time() + 1
    while time.time() < end_time:
        robot_pub.publish(twist2)
def Unstucking():    
	global yaw,cm,cm_width,cm_height
	odom = rospy.wait_for_message("sos_rs1/odom" , Odometry)
	explicit_quat = []
	explicit_quat.append(odom.pose.pose.orientation.x)
    explicit_quat.append(odom.pose.pose.orientation.y)
    explicit_quat.append(odom.pose.pose.orientation.z)
    explicit_quat.append(odom.pose.pose.orientation.w)
    your_euler = tf.transformations.euler_from_quaternion(explicit_quat)
    yaw=your_euler[2] + 0.5*math.pi
    #oriention = tf.transformations.quaternion_from_euler(0,0,(your[2] + 1) % 3)
    SP = Search_Stuck_Point()
    Sx = SP % costmapsize
    Sy = SP // costmapsize
    Rx = cm_width/2
    Ry = cm_height/2
    print(Sx,Sy)
    print(yaw)
    if yaw < 0 :
        yaw = math.pi*2 - yaw
    print(yaw)
    SPAng = math.atan((Sy-Ry)/(signf(Sx-Rx)*(abs(Sx-Rx) + 0.000000001)))
    print(SPAng)
    Rotation = math.pi - yaw + SPAng
    print(Rotation)
	if yaw + SPAng < math.pi :
		Rotation=-Rotation
		cmd_vel(Rotation)
	if yaw + SPAng >= math.pi:
		cmd_vel(Rotation)
    
#def StuckChecker(rn):

def main():
    global twist,robot_pub,cm,sos_rs1
    rospy.init_node('publish_cmd', anonymous=True)
    robot_sub = rospy.Subscriber("/sos_rs1/move_base/local_costmap/costmap",OccupancyGrid, CallCostMap )
    t = threading.Thread(target=Unstucking)
    t.daemon = True
    t.start()
    rospy.spin()
if name == '__main__':
    main()
