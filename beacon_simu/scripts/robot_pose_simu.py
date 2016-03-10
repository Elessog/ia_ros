#!/usr/bin/env python
import rospy,time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
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
    x = np.array([5,-5,np.pi/3,0.5,0.5]);
    dt = 1/30.0;
    u = [0,0];
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
       x=x+f(x,[0,0])*dt
       quat = tf.transformations.quaternion_from_euler(0, 0, x[2]);
       br.sendTransform((x[0], x[1], 0),
                     quat,
                     rospy.Time.now(),
                     base_frame,
                     map_frame)
       msg = PoseStamped()
       msg.header.stamp = rospy.get_rostime()
       msg.header.frame_id = 'map'
       msg.pose.position.x = x[0]
       msg.pose.position.y = x[1]
       msgIntern = Float64MultiArray()
       msgIntern.data.append(u[0]);
       msgIntern.data.append(u[1]);
       #euler = tf.transformations.euler_from_quaternion(quat)
       msgIntern.data.append(x[2]);
       msgIntern.data.append(x[3]);
       msgIntern.data.append(x[4]);
       pub.publish(msg)
       pub_intern.publish(msgIntern)
       time.sleep(dt)

def secondcar():  
    x = np.array([0,0,np.pi/3,0.5,0.6]);
    dt = 1/30.0;
    u = [0,0];
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
       x=x+f(x,[0,0])*dt
       quat = tf.transformations.quaternion_from_euler(0, 0, x[2]);
       br.sendTransform((x[0], x[1], 0),
                     quat,
                     rospy.Time.now(),
                     base_frame,
                     map_frame)
       msg = PoseStamped()
       msg.header.stamp = rospy.get_rostime()
       msg.header.frame_id = 'map'
       msg.pose.position.x = x[0]
       msg.pose.position.y = x[1]
       msgIntern = Float64MultiArray()
       msgIntern.data.append(u[0]);
       msgIntern.data.append(u[1]);
       #euler = tf.transformations.euler_from_quaternion(quat)
       msgIntern.data.append(x[2]);
       msgIntern.data.append(x[3]);
       msgIntern.data.append(x[4]);
       pub.publish(msg)
       pub_intern.publish(msgIntern)
       time.sleep(dt)


def thirdcar():  
    x = np.array([2,2,0,0.5,-0.4]);
    dt = 1/30.0;
    u = [0,0];
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
       x=x+f(x,[0,0])*dt
       quat = tf.transformations.quaternion_from_euler(0, 0, x[2]);
       br.sendTransform((x[0], x[1], 0),
                     quat,
                     rospy.Time.now(),
                     base_frame,
                     map_frame)
       msg = PoseStamped()
       msg.header.stamp = rospy.get_rostime()
       msg.header.frame_id = 'map'
       msg.pose.position.x = x[0]
       msg.pose.position.y = x[1]
       msgIntern = Float64MultiArray()
       msgIntern.data.append(u[0]);
       msgIntern.data.append(u[1]);
       #euler = tf.transformations.euler_from_quaternion(quat)
       msgIntern.data.append(x[2]);
       msgIntern.data.append(x[3]);
       msgIntern.data.append(x[4]);
       pub.publish(msg)
       pub_intern.publish(msgIntern)
       time.sleep(dt)

if __name__ == '__main__':
    beacons = [[-4,4],[0,5],[4,-1],[7,-6],[-7,2]]

    typeMove = 0
    pose_topic = 'robot_pose'
    data_topic = 'robot_data'
    map_frame = "map"
    base_frame = "robot"

    rospy.init_node('robot_simu')

    if rospy.has_param(rospy.get_name()+'/'+'typeMove'):
       typeMove = float(rospy.get_param(rospy.get_name()+'/'+"typeMove"))
    if rospy.has_param(rospy.get_name()+'/'+'pose_topic'):
       pose_topic = rospy.get_param(rospy.get_name()+'/'+"pose_topic")
    if rospy.has_param(rospy.get_name()+'/'+'data_topic'):
       data_topic = rospy.get_param(rospy.get_name()+'/'+"data_topic")
    if rospy.has_param(rospy.get_name()+'/'+'map_frame'):
       map_frame = rospy.get_param(rospy.get_name()+'/'+"map_frame")
    if rospy.has_param(rospy.get_name()+'/'+'base_frame'):
       base_frame = rospy.get_param(rospy.get_name()+'/'+"base_frame")

    pub = rospy.Publisher(pose_topic , PoseStamped, queue_size=10)
    pub_intern = rospy.Publisher(data_topic, Float64MultiArray, queue_size=10)
    if typeMove == 0:
       onecar()
    elif typeMove==1:
       secondcar()
    else:
       thirdcar()
