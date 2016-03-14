#!/usr/bin/env python
import rospy,time,threading,copy,tf
from std_msgs.msg import Float64MultiArray
from ia_msgs.msg import StampedInterval
from math import cos,sin,sqrt,atan2,atan,pi,tan
import numpy as np
from visualization_msgs.msg import Marker
from help_marker import point,line

global robots,pose,start
robots = []
Lock = threading.RLock()

def ShutdownCallback():
    print("end")

def computeCornerPotentiel(x,otherX):
    dist2 = (otherX[0]-x[0])**2+(otherX[1]-x[1])**2
    return (-(otherX[0]-x[0])/dist2,-(otherX[1]-x[1])/dist2,dist2)

def computePotentielForOtherRobot(x,w,robot):
    res = [0,0,0,0]
    res[0] = computeCornerPotentiel(x,[robot.position.x,robot.position.y])
    res[1] = computeCornerPotentiel(x,[robot.position.x+robot.width,robot.position.y])
    res[2] = computeCornerPotentiel(x,[robot.position.x+robot.width,robot.position.y+robot.height])
    res[3] = computeCornerPotentiel(x,[robot.position.x,robot.position.y+robot.height])
    i = 0
    dist = 100
    for r,j in zip(res,range(len(res))):
        if (dist>r[2]):
           dist=r[2]
           i = j
    w[0]+=res[i][0]/4.0
    w[1]+=res[i][1]/4.0

def computePotentielForOtherRobots(x,w):
    global robots
    with Lock:
      for robot in robots:
         computePotentielForOtherRobot(x,w,robot)

def computePotentielForLine(w,x,theta,a,b):
  vABx = b[0]-a[0];
  vABy = b[1]-a[1];
  vAMx = x[0]-a[0];
  vAMy = x[1]-a[1];
  phi = atan2(vABy,vABx)
  eL = (vABx*vAMy-vABy*vAMx)/sqrt(vABx**2+vABy**2);#det([b-a,m-a])/norm(b-a);//distance to the line
  thetabar = phi-atan(eL/2);
  w[0] += cos(thetabar)#tranform direction to follow into uniforme potentiel field to integrate better with obstacle
  w[1] += sin(thetabar)

def potentielToMotorsOrders(x,vold,theta,w):  
  vbar = sqrt(w[0]**2+w[1]**2)
  v = 1*(2/pi)*atan(vbar);
  nThetabar = atan2(w[1],w[0])
  u = (2/pi)*atan(tan((nThetabar-theta)/2));
  return (0.05*(v-vold),u)

def checkPosSegment(a,b,x):
  vOrthoAB = [-(b[1]-a[1]),(b[0]-a[0])]
  vMB = [b[0]-x[0],b[1]-x[1]];
  # we do MB^BC BC is orthogonol to AB M the position of robot
  #print(vOrthoAB,vMB,vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0],vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]>0,a,b,x)
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]>0; #true if we didn't pass through B


def get_ij(case):#matlab convention of multiarray
    i = round(case/nb_grille_X)
    return (i,case-i*nb_grille_X)

def returnCoord(case):
    (i,j) = get_ij(case)
    a = origin+np.array([j*size_gridx,i*size_gridy])
    b = origin+np.array([(j+1)*size_gridx,i*size_gridy])
    c = origin+np.array([(j+1)*size_gridx,(i-1)*size_gridy])
    d = origin+np.array([j*size_gridx,(i-1)*size_gridy])
    return (a,b,c,d)

def goToPoint(dest):
    global x
    p = copy.deepcopy(x)
    followLine(p,dest)

def followLine(deb,end):
    global x,theta,vold
    while(checkPosSegment(deb,end,x)) and not rospy.is_shutdown():
        w =[0,0]
        computePotentielForLine(w,x,theta,deb,end)
        computePotentielForOtherRobots(x,w)
        (v,u) = potentielToMotorsOrders(x,vold,theta,w)
        msg = Float64MultiArray()
        msg.data.append(v)
        msg.data.append(u)
        pub_controller.publish(msg)
        time.sleep(dt)
        

def goto_strategy():
    (a,b,c,d) = returnCoord(assigned_case)
    pa = point(a,1,"map")
    pa.publish(pub_reach)    

    goToPoint(a)
    pc = point(c,3,"map")
    pc.publish(pub_reach)
    followLine(a,c)
    pd = point(d,4,"map")
    pd.publish(pub_reach)
    followLine(c,d)
    pb = point(b,2,"map")
    pb.publish(pub_reach)
    followLine(d,b)

def patrol_strategy():
    division = 6
    i = 0
    dx = size_gridx/division
    (a,b,c,d) = returnCoord(assigned_case)
    while i<division/2:
    ##descend
      delta = np.array([-dx*(2*i),0])
      delta2 = np.array([-dx*(2*i+1),0])
      delta3 = np.array([-dx*(2*i+2),0])
      xn = [(b+delta).tolist()+[0],(c+delta).tolist()+[0],(c+delta2).tolist()+[0],(b+delta2).tolist()+[0],(b+delta3).tolist()+[0]]
      pl = line(xn,4,"map")
      pl.publish(pub_reach)
      followLine(b+delta,c+delta)
    ##left
      followLine(c+delta,c+delta2)
    ##up
      followLine(c+delta2,b+delta2)
    ##left
      followLine(b+delta2,b+delta3)
      i+=1


def box_callback(msg):
    global robots,x,start
    if not test:
       start = True
       x = [msg.data[0].data[0].position.x+msg.data[0].data[0].width/2.0,msg.data[0].data[0].position.y+msg.data[0].data[0].height/2.0]
    with Lock:
      robots = []
      for data,i in zip(msg.data,range(len(msg.data))):
          if i!=0:
             if data.data[0].width<15:
                robots.append(data.data[0])

def data_callback(msg):
    global theta,vold,x,start
    if test:
       start = True
       x=[msg.data[5],msg.data[6]]
    vold = msg.data[3]
    quat = tf.transformations.quaternion_from_euler(0, 0, msg.data[2]);
    euler = tf.transformations.euler_from_quaternion(quat)
    theta = euler[2]

if __name__ == '__main__':

    id_car = 0
    ia_topic = 'robot_ia_data'
    assigned_case = 0
    dt = 1/30.0
    start = False
    rospy.init_node('ia_controller')
    rospy.on_shutdown(ShutdownCallback)
    test = 0

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
    if rospy.has_param(rospy.get_name()+'/'+'testControl'):
       test = float(rospy.get_param(rospy.get_name()+'/'+"testControl"))

    rospy.loginfo("test : %d"%test)
    origin=np.array([x_pos_grid,y_pos_grid])
    size_gridx = x_length_grid/float(nb_grille_X)
    size_gridy = y_length_grid/float(nb_grille_Y)
    pub_controller = rospy.Publisher(controller_topic+"%d"%id_car, Float64MultiArray, queue_size=1)
    pub_reach =  rospy.Publisher("visu_points", Marker, queue_size=1)

    rospy.Subscriber("/ia_slam"+"%d"%id_car+"/toControllerPoses", StampedInterval,box_callback)
    rospy.Subscriber("robot_data"+"%d"%id_car, Float64MultiArray,data_callback)
    while not start and not rospy.is_shutdown():
         rospy.sleep(1.0/2.0)
    rospy.loginfo("start programme")
    #goto_strategy()
    patrol_strategy()
    rospy.loginfo("end programme")
