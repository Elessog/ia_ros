#!/usr/bin/env python
import rospy,time,threading
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from math import cos,sin,sqrt
from visualization_msgs.msg import Marker
import numpy as np
import tf

global robot_position,u
u=[0,0]

def control_callback(msg):
    global u
    u[0] = msg.data[0]
    u[1] = msg.data[1]


def f(x,u):
    xdot=np.array([x[3]*cos(x[4])*cos(x[2]),
    x[3]*cos(x[4])*sin(x[2]),
    x[3]*sin(x[4])/3,
    u[0],
    0])
    return xdot

def f2(x,u):
    xdot=np.array([x[3]*cos(x[2]),
    x[3]*sin(x[2]),
    u[1],
    u[0],
    0])
    return xdot

def onecar(x): 
    global u 
    dt = 1/30.0;
    u = [0,0];
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
       x=x+f2(x,u)*dt
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
       msgIntern.data.append(u[0])
       msgIntern.data.append(u[1])
       msgIntern.data.append(x[2])
       msgIntern.data.append(x[3])
       msgIntern.data.append(x[4])

       ## only used in controller test
       msgIntern.data.append(x[0])
       msgIntern.data.append(x[1])

       pub.publish(msg)
       if show_reach:
         marker.header.stamp = rospy.Time.now();
         pub_reach.publish(marker)
       pub_intern.publish(msgIntern)
       time.sleep(dt)

if __name__ == '__main__':

    typeMove = 0
    pose_topic = 'robot_pose'
    data_topic = 'robot_data'
    map_frame = "map"
    base_frame = "robot"
    show_reach = 1

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
    if rospy.has_param(rospy.get_name()+'/'+'show_reach'):
       show_reach = float(rospy.get_param(rospy.get_name()+'/'+"show_reach"))
    if rospy.has_param(rospy.get_name()+'/'+'controller_topic'):
       controller_topic = rospy.get_param(rospy.get_name()+'/'+"controller_topic")
       if rospy.has_param(rospy.get_name()+'/'+'x_pos'):
          x_pos = rospy.get_param(rospy.get_name()+'/'+"x_pos")
       else:
          rospy.logerr("You must set the initial x_pos")
       if rospy.has_param(rospy.get_name()+'/'+'y_pos'):
          y_pos = rospy.get_param(rospy.get_name()+'/'+"y_pos")
       else:
          rospy.logerr("You must set the initial theta_pos")
       if rospy.has_param(rospy.get_name()+'/'+'theta_pos'):
          theta_pos = rospy.get_param(rospy.get_name()+'/'+"theta_pos")
       else:
          rospy.logerr("You must set the initial theta_pos")
    if rospy.is_shutdown():
       exit(0)

    if show_reach:
       marker = Marker()
       marker.header.frame_id = base_frame;
       marker.header.stamp = rospy.Time.now();
       marker.ns = base_frame;
       marker.id = 0;
       marker.type = 3;
       marker.action = 0;
       marker.pose.position.x = 0;
       marker.pose.position.y = 0;
       marker.pose.position.z = 0;
       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = 1.0;
       marker.scale.x = 15*2;
       marker.scale.y = 15*2;
       marker.scale.z = 0;
       marker.color.a = 0.3
       marker.color.r = 0.0;
       marker.color.g = 1.0;
       marker.color.b = 0.0;

    pub_reach =  rospy.Publisher("visu_reach", Marker, queue_size=1)
    pub = rospy.Publisher(pose_topic , PoseStamped, queue_size=10)
    pub_intern = rospy.Publisher(data_topic, Float64MultiArray, queue_size=10)
    ##
    if typeMove == 0:
       onecar(np.array([5,-5,np.pi/3,0.5,0.5]))
    elif typeMove==1:
       onecar(np.array([0,0,np.pi/3,0.5,0.6]))
    elif typeMove == 2:
       onecar(np.array([2,2,0,0.5,-0.4]))
    else:
       rospy.Subscriber(controller_topic, Float64MultiArray,control_callback)
       onecar(np.array([x_pos,y_pos,theta_pos,0,0]))

