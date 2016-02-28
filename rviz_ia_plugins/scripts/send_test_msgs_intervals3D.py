#!/usr/bin/env python

#import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from ia_msgs.msg import Interval3D,Interv3D
import rospy
from math import cos, sin
import tf

topic = 'test_interval'
publisher = rospy.Publisher( topic, Interval3D, queue_size = 1 )

rospy.init_node( 'test_interval' )

rate = rospy.Rate(10)


interval = Interval3D()
interval.header.frame_id = "/map"
interval.header.stamp = rospy.Time.now()
interv = Interv3D()
interv.position.x=0
interv.position.y=0
interv.position.z=0
interv.width = 1
interv.height = 2
interv.zlevel = 0.5

interval.data.append(interv)
interv = Interv3D()
interv.position.x=2
interv.position.y=2
interv.position.z=0
interv.width = 2
interv.height = 1
interv.zlevel = 1
interval.data.append(interv)


while not rospy.is_shutdown():

    
    publisher.publish( interval )

    interval.data[1].zlevel += 0.01 
    interval.data[0].position.x += 0.01
    rate.sleep()

