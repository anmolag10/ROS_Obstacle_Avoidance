#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan, Imu, NavSatFix, Range
from tf.transformations import euler_from_quaternion
from pyproj import Geod
import numpy as np
import math
import time

left_min = 0
right_min = 0
flag=0
yaw =0 
gps_angle=0
lat1=0
lon1=0

lat2= 49.8998668004
lon2= 8.9001907467

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
    
    
def callback2(data):
    global lat1 
    lat1= data.latitude
    global lon1 
    lon1= data.longitude

def callback3(pose):
    global yaw
    quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

    euler = euler_from_quaternion(quaternion)
    yaw= math.degrees(euler[2]) +180
    yaw = abs(yaw-360)
    yaw = yaw%360





## NODE INTITIALIZATION



rospy.init_node('scan_msg', disable_signals=True)

rospy.Subscriber("/imu", Imu, callback3)
rospy.Subscriber("/fix", NavSatFix, callback2) 
rospy.Subscriber("/laser_scan",LaserScan, callback)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

speed = Twist()
r = rospy.Rate(100)

while not rospy.is_shutdown():
    geodesic =Geod(ellps='WGS84')
    bearing, reverse_bearing, dist = geodesic.inv(lon1,lat1,lon2,lat2)
    bearing = bearing 
    angle_diff =bearing-yaw
    print("DISTANCE: ", dist)
    if dist<0.8:
        speed.linear.x = 0
        speed.angular.z = 0
        print("stop")
        pub.publish(speed)

    elif right_min>3 and left_min>3:
        if angle_diff>0:
            speed.angular.z = -0.2
            print("aligning right")
        elif angle_diff<0:
            speed.angular.z = +0.2
            print("aligning left")
        print("straight")
        speed.linear.x = -0.3
        pub.publish(speed)

    elif left_min> right_min:
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
        #print(speed)
    
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
       # print(speed)
    
    r.sleep()

