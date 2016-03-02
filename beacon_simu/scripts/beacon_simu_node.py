#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from ia_msgs.msg import BeaconDist
import math as m

global position,beacons,start
start = False

def callback(data):
    global position,start,stamp
    position = data.pose.position
    stamp = data.header
    start = True

def talker():
    global position,beacons,stamp
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if start:
          for beacon,i in zip(beacons,range(len(beacons))): 
            pub.publish(stamp,i,m.sqrt((beacon[0]-position.x)**2+(beacon[1]-position.y)**2))
            rate.sleep()
        else:
          rate.sleep()

if __name__ == '__main__':
    beacons = [[-4,4],[0,5],[4,-1],[7,-6],[-7,2]]

    rospy.init_node('beacon_simu')
    rospy.Subscriber("robot_pose", PoseStamped, callback)
    pub = rospy.Publisher('beacon_echo', BeaconDist, queue_size=10)
    talker()
