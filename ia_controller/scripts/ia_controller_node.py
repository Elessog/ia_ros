#!/usr/bin/env python
import rospy,time,threading
from std_msgs.msg import Float64MultiArray
from math import cos,sin,sqrt
import numpy as np

def callback():
    

if __name__ == '__main__':

    id_car = 0
    ia_topic = 'robot_ia_data'
    assigned_case = 0

    rospy.init_node('ia_controller')

    if rospy.has_param(rospy.get_name()+'/'+'id'):
       id_car = float(rospy.get_param(rospy.get_name()+'/'+"id"))
    if rospy.has_param(rospy.get_name()+'/'+'nb_grille_X'):
       nb_grille_X = float(rospy.get_param(rospy.get_name()+'/'+"nb_grille_X"))
    if rospy.has_param(rospy.get_name()+'/'+'nb_grille_Y'):
       nb_grille_Y = float(rospy.get_param(rospy.get_name()+'/'+"nb_grille_Y"))
    if rospy.has_param(rospy.get_name()+'/'+'assigned_case'):
       assigned_case = float(rospy.get_param(rospy.get_name()+'/'+"assigned_case"))
    if rospy.has_param(rospy.get_name()+'/'+'ia_topic'):
       ia_topic = rospy.get_param(rospy.get_name()+'/'+"ia_topic")
    if rospy.has_param(rospy.get_name()+'/'+'controller_topic'):
       controller_topic = rospy.get_param(rospy.get_name()+'/'+"controller_topic")


    pub_controller = rospy.Publisher(controller_topic+str(id_car), Float64MultiArray, queue_size=1)
    ##

    rospy.Subscriber(ia_topic, Float64MultiArray,control_callback)
    rospy.spin()

