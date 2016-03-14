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
    if (!nh_private_.getParam ("max_past_iter", max_past_iter_))
        max_past_iter_ = 1000;
    if (!nh_private_.getParam ("gps_precision", gps_precision_))
        gps_precision_ = 1;
    if (!nh_private_.getParam ("division_box_rate", division_box_))
        division_box_ = 0.5;
    if (!nh_private_.getParam ("extern_robot_topic", external_interv_topic_))
        external_interv_topic_ = "extern_robot";
    if (!nh_private_.getParam ("heading_precision", heading_precision_))
        heading_precision_ = 0.01;
    if (!nh_private_.getParam ("speed_precision", speed_precision_))
        speed_precision_ = 0.05;
    if (!nh_private_.getParam ("headingWheel_precision", headingWheel_precision_))
        headingWheel_precision_ = 0.01;
    if (!nh_private_.getParam ("id_robot", id_robot_))
        id_robot_ = 0;
    if (!nh_private_.getParam ("quick_start", quickstart_))
        quickstart_ = true;
    
    service_ = nh_private_.advertiseService("starter", &IaSlam::starterControl,this);
    beacon_sub_ = nh_.subscribe(beacon_topic_,1, &IaSlam::beaconDist,this);
    internal_sub_ = nh_.subscribe(internal_topic_,1, &IaSlam::internRobot,this);
    external_interv_sub_ = nh_.subscribe(external_interv_topic_,1, &IaSlam::betweenRobot,this);

    beacon_pub_ = nh_private_.advertise<ia_msgs::StampedInterval>("beacons", 2);
    toController_pub_ = nh_private_.advertise<ia_msgs::StampedInterval>("toControllerPoses", 2);
    position_pub_ = nh_private_.advertise<ia_msgs::Interval>("pose", 10);
    state_vector = new IntervalVector(4,Interval(0,0));
    dstate_vector = new IntervalVector(4,Interval(-50,50));
    temp_contract_vector = new IntervalVector(11,Interval(-50,50));
    u = new IntervalVector(2,Interval(-1,1));

///////////////////////////////// Constraints ////////////////////////////////////////////////////
    x = new Variable();  //robot interval pose
    y = new Variable();
    ax = new Variable(); //beacon interval
    ay = new Variable();
    d = new Variable();//distance to beacons
    uX = new Variable(4);//state vector at time t+1
    ud = new Variable(2);//input on actuators
    X = new Variable(4);//state vector at time t
    idt = new Variable();//interval for dt 
    dt = 1/float(ros_rate);

    distfunc= new Function(*x,*y,*ax,*ay,*d,sqr(*x-*ax)+sqr(*y-*ay)-sqr(*d)); // distance to beacon constraint
    updfunc= new Function(*X,*uX,*ud,*idt,Return(                             // state equation constraints
                                 (*X)[0] + (*X)[3]*cos((*X)[2])*(*idt) - (*uX)[0],
                                 (*X)[1] + (*X)[3]*sin((*X)[2])*(*idt) - (*uX)[1],
                                 (*X)[2] + (*ud)[1]*(*idt) - (*uX)[2],
                                 (*X)[3] + (*ud)[0]*(*idt) - (*uX)[3]
                                 ));
    c = new CtcFwdBwd(*distfunc);
    distContract =new CtcFixPoint(*c,1e-01);
    distCut =new Ctc3BCid(*distContract);
    
    s =new CtcFwdBwd(*updfunc);
    updContract =new CtcFixPoint(*s,1e-01);
    //updContract =new Ctc3BCid(*fs);
/////////////////////////////////////////////////////////////////////////////////////////////////
    tf::TransformListener listener;
    tf::StampedTransform transform;
    double roll, pitch, yaw;
    try{// get fisrt position (simulating start of mission with first known position )
      listener.waitForTransform(map_frame_, base_frame_,
                              ros::Time::now(), ros::Duration(3.0));
      listener.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      (*state_vector)[0] = Interval(transform.getOrigin().x()).inflate(gps_precision_);
      (*state_vector)[1] = Interval(transform.getOrigin().y()).inflate(gps_precision_);
      (*state_vector)[2] = Interval(yaw).inflate(0.1);
      (*state_vector)[3] = Interval(1).inflate(0.1);
      start = true;
      lastIter = ros::Time::now();
      contractTime = ros::Time::now();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::shutdown();
      exit(-1);
    }
}

IaSlam::~IaSlam(){
   delete state_vector;
   delete dstate_vector;
   delete u;
   delete temp_contract_vector; 
   delete distContract;
   delete c;
   delete updContract;
   delete s;
   delete compo;
   delete x;
   delete y;
   delete ax;
   delete ay;
   delete d;
   delete uX;
   delete ud;
   delete X;
   delete idt;
   delete distfunc;
   delete updfunc;
   for (auto it=past.begin(); it!=past.end();++it){
      delete ((*it).first);
      for (auto itt=(*it).second.begin();itt!=(*it).second.end();++itt){
           delete ((*itt).first);
      }
   }
   for (auto it=landmarksMap.begin(); it!=landmarksMap.end();++it){
      for (auto itt=(*it).second.begin();itt!=(*it).second.end();++itt){
           delete *itt;
      }
   }
   for (auto it=otherRobotMap.begin(); it!=otherRobotMap.end();++it){
      delete (*it).second;
   }
} 

void IaSlam::dump(){
  ofstream fichier;
  std::string filepath = "~/data/dump.txt";
  fichier.open(filepath.c_str(), ios::out | ios::trunc);
  for (int i=0 ; i<past.size();i++){
    fichier << i << " " << pastDt[i] << ' ' <<  (*(past[i].first))[0].lb() << ' ' 
            << (*(past[i].first))[0].ub() << ' ' <<  (*(past[i].first))[1].lb()
            << ' ' <<  (*(past[i].first))[1].ub()<< std::endl;
  }
  fichier.flush();  
  fichier.close();
  ros::shutdown();
  exit(0);
}



bool IaSlam::starterControl(ia_msgs::Start_Slam::Request &req,ia_msgs::Start_Slam::Response &res){
  start = req.demand; 
  res.result = true;
  return true;
}

void IaSlam::spin(){
  ros::Rate loop_rate(ros_rate);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (start)
       ia_iter();
  }
}

void IaSlam::updateOtherRobot(double dtt){
   for (auto it=otherRobotMap.begin();it!=otherRobotMap.end();++it){
     double vmax=1;
     (*((*it).second))[0] += Interval(-vmax,vmax)*dtt;
     (*((*it).second))[1] += Interval(-vmax,vmax)*dtt;
  }
};


}//end namespace

using namespace ia_slam;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"ia_slam");
    
    IaSlam node;

    node.spin();

    return 0;
}
