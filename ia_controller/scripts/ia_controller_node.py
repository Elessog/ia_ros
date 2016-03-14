#!/usr/bin/env python
import rospy,time
from std_msgs.msg import Float64MultiArray
from math import cos,sin,sqrt
import numpy as np

global robots,pose

def calPotR(x,otherX):
    dist2 = (otherX[0]-x[0])**2+(otherX[1]-x[1])**2
    return (-(otherX[0]-x[0])/dist2,-(otherX[1]-x[1])/dist2,dist2)

def calPotRobots(x,w,robot):
    res = [0,0,0,0]
    res[0] = calcPotR(x,[robot.position.x,robot.position.y])
    res[1] = calcPotR(x,[robot.position.x+robot.width,robot.position.y])
    res[2] = calcPotR(x,[robot.position.x+robot.width,robot.position.y+robot.height])
    res[3] = calcPotR(x,[robot.position.x,robot.position.y+robot.height])
    i = 0
    dist = 0
    for r,j in zip(res,range(len(res))):
        if (dist<r[3]):
           dist=r[3]
           i = j
    w[0]+=res[i][0]
    w[1]+=res[i][1]

def calPotRobots(x,w):
    global robots
    for robot in robots:
       calcPotRobot(x,w,robot)

def calPotLine(w,x,xold,theta,a,b):
  vABx = b[0]-a[0];
  vABy = b[1]-a[1];
  vAMx = x[0]-a[0];
  vAMy = x[1]-a[1];
  phi = atan2(vABy,vABx)
  eL = (vABx*vAMy-vABy*vAMx)/sqrt(vABx**2+vABy**2);#det([b-a,m-a])/norm(b-a);//distance to the line
  thetabar = phi-atan(eL/2);
  #e = thetabar-theta;
  w[0] += cos(thetabar)#tranform direction to follow into uniforme potentiel field to integrate better with obstacle
  w[1] += sin(thetabar)

def calPot(x,vold,theta,w):  
  vbar = sqrt(w[0]**2+w[1]**2)
  v = 1*(2/pi)*atan(vbar - vold);
  nThetabar = atan2(w[1],w[0])
  u = (2/pi)*atan(tan((nThetabar-theta)/2));
  return (v,u)

def checkPosSegment(a,b,x):
  vOrthoAB = [-(b[1]-a[1]),(b[0]-a[1])]
  vMB = [b[0]-x[0],b[1]-x[1]];
  # we do MB^BC BC is orthogonol to AB M the position of robot
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]<0; #true if we didn't pass through B


def get_ij(case):#matlab convention of multiarray
    i = round(case/nb_grille_X)
    return (i,case-i*nb_grille_X)

def returnCoord(case):
    (i,j) = get_ij(case)
    a = origin+np.array([j*size_gridx,i*size_gridy])
    b = origin+np.array([(j+1)*size_gridx,i*size_gridy])
    c = origin+np.array([(j+1)*size_gridx,(i+1)*size_gridy])
    d = origin+np.array([j*size_gridx,(i+1)*size_gridy])
    return (a,b,c,d)

def goToPoint(dest)
    p = x
    followLine(p,dest)

def followLine(deb,end):
    while(checkPosSegment(deb,end,x)):
        w =[0,0]
        calPotLine(w,x,xold,theta,deb,end)
        calPotRobots(x,w)
        (u,v) = calPot(x,vold,theta,w)
        pub_controller.publish(u,v)
        time.sleep(dt)
        

def goto_strategy():
    (a,b,c,d) = returnCoord(assigned_case)
    goToPoint(a)
    followLine(a,c)
    followLine(c,d)
    followLine(d,b)

def patrol_strategy():
    division = 3
    i = 0
    dx = size_gridx/division
    while i<3:
    ##descend
      delta = np.array([-dx*(2*i),0])
      delta2 = np.array([-dx*(2*i+1),0])
      delta3 = np.array([-dx*(2*i+2),0])
      followLine(b+delta,c+delta)
    ##left
      followLine(c+delta,c+deltap)
    ##up
      followLine(c+delta2,b+delta2)
    ##left
      followLine(b+delta3,b+delta3)


def control_callback():
    

if __name__ == '__main__':

    id_car = 0
    ia_topic = 'robot_ia_data'
    assigned_case = 0
    dt = 1/30.0

    rospy.init_node('ia_controller')

    if rospy.has_param(rospy.get_name()+'/'+'id'):
       id_car = float(rospy.get_param(rospy.get_name()+'/'+"id"))
    if rospy.has_param(rospy.get_name()+'/'+'nb_grille_X'):
       nb_grille_X = float(rospy.get_param(rospy.get_name()+'/'+"nb_grille_X"))
    if rospy.has_param(rospy.get_name()+'/'+'nb_grille_Y'):
       nb_grille_Y = float(rospy.get_param(rospy.get_name()+'/'+"nb_grille_Y"))
    if rospy.has_param(rospy.get_name()+'/'+'x_pos_grid'):
       x_pos_grid = float(rospy.get_param(rospy.get_name()+'/'+"x_pos_grid"))
    if rospy.has_param(rospy.get_name()+'/'+'y_pos_grid'):
       y_pos_grid = float(rospy.get_param(rospy.get_name()+'/'+"y_pos_grid"))
    if rospy.has_param(rospy.get_name()+'/'+'x_length_grid'):
       x_length_grid = float(rospy.get_param(rospy.get_name()+'/'+"x_length_grid"))
    if rospy.has_param(rospy.get_name()+'/'+'y_length_grid'):
       y_length_grid = float(rospy.get_param(rospy.get_name()+'/'+"y_length_grid"))
    if rospy.has_param(rospy.get_name()+'/'+'assigned_case'):
       assigned_case = float(rospy.get_param(rospy.get_name()+'/'+"assigned_case"))
    if rospy.has_param(rospy.get_name()+'/'+'ia_topic'):
       ia_topic = rospy.get_param(rospy.get_name()+'/'+"ia_topic")
    if rospy.has_param(rospy.get_name()+'/'+'controller_topic'):
       controller_topic = rospy.get_param(rospy.get_name()+'/'+"controller_topic")

    origin=np.array([x_pos_grid,y_pos_grid])
    size_gridx = x_length_grid/float(nb_grille_X)
    size_gridy = y_length_grid/float(nb_grille_Y)
    pub_controller = rospy.Publisher(controller_topic+str(id_car), Float64MultiArray, queue_size=1)
    ##

    rospy.Subscriber("/ia_slam"+str(id_car)+"/toControllerPoses", Float64MultiArray,control_callback)
    rospy.spin()

