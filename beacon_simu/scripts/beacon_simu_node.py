#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from ia_msgs.msg import BeaconDist,DistInterBeacon
import math as m

global position,beacons,start,poses
start = False

def callback(data):
    global position,start,stamp
    position = data.pose.position
    stamp = data.header
    start = True

def talker():
    global position,beacons,stamp,poses
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if start:
          for beacon,i in zip(beacons,range(len(beacons))): 
            dist  = m.sqrt((beacon[0]-position.x)**2+(beacon[1]-position.y)**2);
            if dist < 10:
              pub.publish(stamp,i,dist,distBeacons[i])
            rate.sleep()
          poses.header = stamp
          if publish_beacons:
             pubReal.publish(poses)
        else:
          rate.sleep()
        

if __name__ == '__main__':
    beacons = [[-4,4],[0,5],[4,-1],[-7,-6],[-7,2]]

    rospy.init_node('beacon_simu')

    pose_topic = 'robot_pose'
    beacon_topic = 'beacon_echo'
    publish_beacons = 1

    if rospy.has_param(rospy.get_name()+'/'+'pose_topic'):
       pose_topic = rospy.get_param(rospy.get_name()+'/'+"pose_topic")
    if rospy.has_param(rospy.get_name()+'/'+'beacon_topic'):
       beacon_topic = rospy.get_param(rospy.get_name()+'/'+"beacon_topic")
    if rospy.has_param(rospy.get_name()+'/'+'publish_beacons'):
       publish_beacons = float(rospy.get_param(rospy.get_name()+'/'+"publish_beacons"))


    rospy.Subscriber(pose_topic, PoseStamped, callback)
    pub = rospy.Publisher(beacon_topic, BeaconDist, queue_size=10)
    pubReal = rospy.Publisher('beacon_poses',PointCloud, queue_size=10)
    poses = PointCloud();
    distBeacons = []
    for beacon in beacons:
       poses.points.append(Point32(beacon[0],beacon[1],0)) 
       distBeacon = []
       for beac,i in zip(beacons,range(len(beacons))):
           dist = m.sqrt((beacon[0]-beac[0])**2+(beacon[1]-beac[1])**2)
           if dist <= 10 and not dist == 0:
              distBeacon.append(DistInterBeacon(i,dist))
       distBeacons.append(distBeacon)
    talker()
    
