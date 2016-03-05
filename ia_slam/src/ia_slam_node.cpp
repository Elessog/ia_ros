#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{

IaSlam::IaSlam():start(false)
{
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh_;

    if (!nh_private_.getParam ("map_frame", map_frame_))
        map_frame_ = "/map";
    if (!nh_private_.getParam ("robot_frame", base_frame_))
        base_frame_ = "/robot";
    if (!nh_private_.getParam ("beacon_topic", beacon_topic_))
        beacon_topic_= "beacon_echo";
    if (!nh_private_.getParam ("internal_topic", internal_topic_))
        internal_topic_= "robot_data";
    if (!nh_private_.getParam ("sensorPrecision", sensorPrecision_))
        sensorPrecision_ = 0.2;
    if (!nh_private_.getParam ("ros_rate", ros_rate))
        ros_rate = 30;
    if (!nh_private_.getParam ("nb_outliers", nb_outliers_))
        nb_outliers_ = 3;
    if (!nh_private_.getParam ("max_box", max_box_))
        max_box_ = 10;
    if (!nh_private_.getParam ("gps_precision", gps_precision_))
        gps_precision_ = 1;
    
    
    service_ = nh_private_.advertiseService("starter", &IaSlam::starterControl,this);
    beacon_sub_ = nh_.subscribe(beacon_topic_,1, &IaSlam::beaconDist,this);
    internal_sub_ = nh_.subscribe(internal_topic_,1, &IaSlam::internRobot,this);


    beacon_pub_ = nh_private_.advertise<ia_msgs::Interval>("beacons", 10);
    position_pub_ = nh_private_.advertise<ia_msgs::Interval>("pose", 10);
    state_vector = new IntervalVector(5,Interval(0,0));
    dstate_vector = new IntervalVector(5,Interval(-50,50));
    temp_contract_vector = new IntervalVector(12,Interval(-50,50));
    u = new IntervalVector(2,Interval(-1,1));
    x = new Variable();
    y = new Variable();
    ax = new Variable();
    ay = new Variable();
    d = new Variable();
    uX = new Variable(5);
    ud = new Variable(2);
    X = new Variable(5);
    dt = 1/float(ros_rate);
    distfunc= new Function(*x,*y,*ax,*ay,*d,sqr(*x-*ax)+sqr(*y-*ay)-sqr(*d));
    updfunc= new Function(*X,*uX,*ud,Return(
                                 (*X)[0] + (*X)[3]*cos((*X)[4])*cos((*X)[2])*dt - (*uX)[0],
                                 (*X)[1] + (*X)[3]*cos((*X)[4])*sin((*X)[2])*dt - (*uX)[1],
                                 (*X)[2] + (*X)[3]*sin((*X)[4])*dt/3.0 - (*uX)[2],
                                 (*X)[3] + (*ud)[0]*dt - (*uX)[3],
                                 (*X)[4] + (*ud)[1]*dt - (*uX)[4]
                                 ));
    c =new CtcFwdBwd(*distfunc);
    distContract =new CtcFixPoint(*c,1e-01);
    s =new CtcFwdBwd(*updfunc);
    updContract =new CtcFixPoint(*s,1e-01);
    
    tf::TransformListener listener;
    tf::StampedTransform transform;
    double roll, pitch, yaw;
    try{
      listener.waitForTransform(map_frame_, base_frame_,
                              ros::Time::now(), ros::Duration(3.0));
      listener.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      (*state_vector)[0] = Interval(transform.getOrigin().x()).inflate(gps_precision_);
      (*state_vector)[1] = Interval(transform.getOrigin().y()).inflate(gps_precision_);
      (*state_vector)[2] = Interval(yaw).inflate(0.1);
      (*state_vector)[3] = Interval(2).inflate(0.1);
      start = true;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      exit(-1);
    }
}

IaSlam::~IaSlam(){} 

}//end namespace

using namespace ia_slam;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"ia_slam");
    
    IaSlam node;

    node.spin();

    return 0;
}
