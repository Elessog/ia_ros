#!/usr/bin/env python
import rospy,time
from geometry_msgs.msg import PoseStamped
from math import cos,sin
import numpy as np
import tf

global robot_position,beacons

def f(x,u):
    xdot=np.array([x[3]*cos(x[4])*cos(x[2]),
    x[3]*cos(x[4])*sin(x[2]),
    x[3]*sin(x[4])/3,
    u[0],
    u[1]])
    return xdot

def onecar():  
    x = np.array([5,-5,np.pi/3,2,0.5]);
    dt = 0.01;
    u = [0,0];
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
       x=x+f(x,[0,0])*dt
       br.sendTransform((x[0], x[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, x[2]),
                     rospy.Time.now(),
                     "robot",
                     "map")
       msg = PoseStamped()
       msg.header.stamp = rospy.get_rostime()
       msg.header.frame_id = 'map'
       msg.pose.position.x = x[0]
       msg.pose.position.y = x[1]
       pub.publish(msg)
       time.sleep(dt)

if __name__ == '__main__':
    beacons = [[-4,4],[0,5],[4,-1],[7,-6],[-7,2]]

    rospy.init_node('robot_simu')
    pub = rospy.Publisher('robot_pose', PoseStamped, queue_size=10)
    onecar()
