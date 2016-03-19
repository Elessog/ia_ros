#!/usr/bin/env python
import rospy,time
from geometry_msgs.msg import PoseStamped
from ia_msgs.msg import StampedInterval,Interv
import numpy as np
import tf,copy
import math as m

class robot_handler(object):
     def __init__(self,number):
         self.id = number
         self.time = rospy.Time.now()
         self.begin = False
         self.pub = rospy.Publisher(base_topic_send+str(i+1) , StampedInterval, queue_size=1)
         rospy.Subscriber("%s%d"%(pose_topic,self.id), PoseStamped, self.pose_callback)
         rospy.Subscriber("%s%d/beacons"%(base_topic_rcv,self.id), StampedInterval, self.beacons_callback)

     def get_id(self):
         return self.id

     def pose_callback(self,msg):
         self.position = msg.pose.position
         self.begin  = True

     def beacons_callback(self,msg):
         if (msg.header.stamp-self.time).to_sec() > 3 and self.begin:
            self.time = msg.header.stamp
            self.check_send(msg)

     def check_send(self,msg):
         for handler in  handlers:
            if (handler.is_close_enough(self.position) and not self.id == handler.get_id()):
               if len(msg.data) > 0:
                msgT =  copy.deepcopy(msg)
                msgT.header.stamp = rospy.Time.now()
                msgT.data[0].data[1].width = m.sqrt(handler.get_distance2(self.position))
                handler.send(msgT)

     def get_distance2(self,position):
         return (position.x-self.position.x)**2+(position.y-self.position.y)**2 

     def is_close_enough(self,position):
         if not self.begin:
            return False
         return self.get_distance2(position) < limit_distance**2          

     def send(self,msg):
         self.pub.publish(msg)

if __name__ == '__main__':
    robot_number = 3
    pose_topic = 'robot_pose'
    base_topic_send = 'extern_robot'
    base_topic_rcv = 'ia_slam'
    limit_distance = 10
    rospy.init_node('exchange_robot_simu')

    if rospy.has_param(rospy.get_name()+'/'+'robot_number'):
       robot_number = float(rospy.get_param(rospy.get_name()+'/'+"robot_number"))
    if rospy.has_param(rospy.get_name()+'/'+'robot_topic_base_send_name'):
       base_topic_send = rospy.get_param(rospy.get_name()+'/'+"robot_topic_base_send_name")
    if rospy.has_param(rospy.get_name()+'/'+'robot_topic_base_rcv_name'):
       base_topic_rcv = rospy.get_param(rospy.get_name()+'/'+"robot_topic_base_rcv_name")
    if rospy.has_param(rospy.get_name()+'/'+'pose_topic_base'):
       map_frame = rospy.get_param(rospy.get_name()+'/'+"pose_topic_base")

    handlers = []
    for i in range(robot_number):
       handlers.append(robot_handler(i+1))
    rospy.spin()
