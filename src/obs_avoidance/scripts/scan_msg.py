#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan 
import numpy as np
import math
import time

left_min = 0
right_min = 0
flag=0

##CALLBACK
def callback(msg):
    global left_min, right_min
    ranges = msg.ranges
    ranges = [30 if x== float('inf') else x for x in ranges]
    right = ranges[0:360]
    left = ranges[360:720]
    left.sort()
    left_min =left[0]
    right.sort()
    right_min = right[0]
    
    


## NODE INTITIALIZATION
rospy.init_node('scan_msg', disable_signals=True)

rospy.Subscriber("/laser_scan",LaserScan, callback)

pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

speed = Twist()
r = rospy.Rate(100)

while not rospy.is_shutdown():
    
    if right_min>5 or left_min>5:
        print("straight")
        speed.linear.x = -0.3
        speed.angular.z = 0
    pub.publish(speed)

    if left_min> right_min:
        ##left
        print("left")
        if flag == 0:
            initial = right_min
            flag=1
        
        diff = initial - right_min
        final = initial +diff

        speed.angular.z = final/15
        speed.linear.x = -right_min/12
        pub.publish(speed)     
        print(speed)
    
    elif right_min> left_min:
        ##right
        print("right")
        if flag == 0:
            initial = left_min
            flag=1

        diff = initial - left_min
        final = initial +diff

        speed.angular.z = -final/15
        speed.linear.x = -left_min/12
        pub.publish(speed)
        print(speed)


    r.sleep()

