#!/usr/bin/env python

#import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from ia_msgs.msg import Interval,Interv
import rospy
from math import cos, sin
import tf

topic = 'test_interval'
publisher = rospy.Publisher( topic, Interval, queue_size = 1 )

rospy.init_node( 'test_interval' )

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
radius = 5
angle = 0

dist = 3

interval = Interval()
interval.header.frame_id = "/map"
interval.header.stamp = rospy.Time.now()
interv = Interv()
interv.position.x=0
interv.position.y=0
interv.position.z=0
interv.width = 1
interv.height = 2


interval.data.append(interv)
interv2 = Interv()
interv2.position.x=2
interv2.position.y=2
interv2.position.z=0
interv2.width = 2
interv2.height = 1
interval.data.append(interv2)


while not rospy.is_shutdown():

    
    publisher.publish( interval )

    interval.data[0].width += 0.01 
    interval.data[1].position.x += 0.01
    rate.sleep()

