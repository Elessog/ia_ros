from visualization_msgs.msg import Marker
import rospy
from geometry_msgs.msg import Point32

class point():
   def __init__(self,x,id_,frame):
       self.marker = Marker()
       self.marker.header.frame_id = frame;
       self.marker.header.stamp = rospy.Time.now();
       self.marker.ns = "r";
       self.marker.id = id_;
       self.marker.type = 3;
       self.marker.action = 0;
       self.marker.pose.position.x = x[0];
       self.marker.pose.position.y = x[1];
       self.marker.pose.position.z = 0;
       self.marker.pose.orientation.x = 0.0;
       self.marker.pose.orientation.y = 0.0;
       self.marker.pose.orientation.z = 0.0;
       self.marker.pose.orientation.w = 1.0;
       self.marker.scale.x = 0.5;
       self.marker.scale.y = 0.5;
       self.marker.scale.z = 0;
       self.marker.color.a = 0.3
       self.marker.color.r = 0.0;
       self.marker.color.g = 1.0;
       self.marker.color.b = 0.0;
   def publish(self,publisher):
       publisher.publish(self.marker)


class line():
   def __init__(self,x,id_,frame):
       self.marker = Marker()
       self.marker.header.frame_id = frame;
       self.marker.header.stamp = rospy.Time.now();
       self.marker.ns = "r";
       self.marker.id = id_;
       self.marker.type = 4;
       self.marker.action = 0;
       z = []
       for xp in x:
          z.append(Point32(xp[0],xp[1],xp[2]))
       self.marker.points = z
       self.marker.scale.x = 0.1;
       self.marker.color.a = 0.3
       self.marker.color.r = 0.0;
       self.marker.color.g = 1.0;
       self.marker.color.b = 0.0;
   def publish(self,publisher):
       publisher.publish(self.marker)
