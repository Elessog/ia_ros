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
<<<<<<< HEAD
    if (!nh_private_.getParam ("gps_precision", gps_precision_))
        gps_precision_ = 1;
    
=======
>>>>>>> 23c9ff033674454510a912db6b338eb85e326248
    
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
<<<<<<< HEAD
      (*state_vector)[0] = Interval(transform.getOrigin().x()).inflate(gps_precision_);
      (*state_vector)[1] = Interval(transform.getOrigin().y()).inflate(gps_precision_);
=======
      (*state_vector)[0] = Interval(transform.getOrigin().x()).inflate(2);
      (*state_vector)[1] = Interval(transform.getOrigin().y()).inflate(2);
>>>>>>> 23c9ff033674454510a912db6b338eb85e326248
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

<<<<<<< HEAD
=======
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

bool IaSlam::starterControl(ia_msgs::Start_Slam::Request &req,ia_msgs::Start_Slam::Response &res){
  start = req.demand; 
  res.result = true;
  return true;
}

void IaSlam::beaconDist(const ia_msgs::BeaconDist msg)
{
  start = true;
  map[msg.id] = std::make_pair(Interval(msg.distance).inflate(sensorPrecision_),true);
  std::map<int,std::vector< IntervalVector*> >::iterator itL;
  IntervalVector contract_vector(5);
  itL = landmarksMap.find(msg.id);
  if (itL != landmarksMap.end())
  {
     for (auto it=msg.distBeacons.cbegin();it!=msg.distBeacons.cend();it++)
     {
        std::map<int,std::vector< IntervalVector*> >::iterator itLL;
        itLL = landmarksMap.find((*it).id);
        if (itLL != landmarksMap.end())
        {
           for (auto baseInt = landmarksMap[msg.id].begin();baseInt!=landmarksMap[msg.id].end();baseInt++)
           {
               for (auto otherInt = landmarksMap[(*it).id].begin();otherInt!=landmarksMap[(*it).id].end();otherInt++)
               { 
                  (contract_vector)[0] = (**baseInt)[0];
                  (contract_vector)[1] = (**baseInt)[1];
                  (contract_vector)[2] = (**otherInt)[0];
                  (contract_vector)[3] = (**otherInt)[1];
                  (contract_vector)[4] = Interval((*it).dist).inflate(0.2);
                  distContract->contract(contract_vector);
                  (**baseInt)[0] = (contract_vector)[0];
                  (**baseInt)[1] = (contract_vector)[1];
                  (**otherInt)[0] = contract_vector[2];
                  (**otherInt)[1] = contract_vector[3];
               }
           }
        }
     }
  }
}

void IaSlam::internRobot(const std_msgs::Float64MultiArray msg)
{
  data_robot_.clear();
  for (auto it = msg.data.cbegin();it!=msg.data.cend();it++)
     data_robot_.push_back(*it);
}

void IaSlam::ia_iter(){
     updateState();
     std::vector<std::pair<Interval*,int> > beacs;
     for (auto it = map.begin(); it != map.end(); ++it){
       beacs.push_back(std::make_pair(new Interval((*it).second.first),(*it).first));
     }
     IntervalVector *state = new IntervalVector(*state_vector);
     std::pair<IntervalVector*,std::vector<std::pair<Interval*,int> > > newMeas =  std::make_pair(state,beacs);
     past.push_back(newMeas);
     map.clear();
     if (beacs.size()>0){
        contractPast();
        //ROS_INFO("Number of beacons %d",(int) beacs.size());
     }
     publishInterval();
}

void IaSlam::publishInterval(){
   ia_msgs::Interval position_msg;
   ia_msgs::Interval beacons_msg;
   position_msg.header.stamp =  ros::Time::now();
   beacons_msg.header.stamp =  ros::Time::now();
   position_msg.header.frame_id = map_frame_;
   beacons_msg.header.frame_id = map_frame_;
   for (auto it = landmarksMap.cbegin(); it != landmarksMap.cend(); ++it)
       intervalToMsg(beacons_msg,(*it).second);
   ia_msgs::Interv newPoint;
   newPoint.position.x = (*state_vector)[0].lb();
   newPoint.position.y = (*state_vector)[1].lb();
   newPoint.position.z = 0;
   newPoint.width = (*state_vector)[0].diam();
   newPoint.height = (*state_vector)[1].diam();
   position_msg.data.push_back(newPoint);
   beacon_pub_.publish(beacons_msg);
   position_pub_.publish(position_msg);
}

void IaSlam::intervalToMsg(ia_msgs::Interval &interv,const std::vector<IntervalVector*> &boxes){
     for (auto box = boxes.cbegin();box!=boxes.cend();box++)
     { 
       ia_msgs::Interv newPoint;
       newPoint.position.x = (*(*box))[0].lb();
       newPoint.position.y = (*(*box))[1].lb();
       newPoint.position.z = 0;
       newPoint.width = (*(*box))[0].diam();
       newPoint.height = (*(*box))[1].diam();
       interv.data.push_back(newPoint);
     }
}

void IaSlam::updateState(){
     (*dstate_vector)[0] = (*state_vector)[3]*cos((*state_vector)[4])*cos((*state_vector)[2]);//x
     (*dstate_vector)[1] = (*state_vector)[3]*cos((*state_vector)[4])*sin((*state_vector)[2]);//y
     (*dstate_vector)[2] = (*state_vector)[3]*sin((*state_vector)[4])/3.0;//thetap
     (*dstate_vector)[3] = (*u)[0]&(Interval(data_robot_[0]).inflate(0.1));//acc
     (*dstate_vector)[4] = (*u)[1]&(Interval(data_robot_[1]).inflate(0.1));//deltap
     (*state_vector)[0] += (*dstate_vector)[0]*dt;
     (*state_vector)[1] += (*dstate_vector)[1]*dt;
     (*state_vector)[2] += (*dstate_vector)[2]*dt;
     (*state_vector)[3] += (*dstate_vector)[3]*dt;
     (*state_vector)[4] += (*dstate_vector)[4]*dt;
     Interval heading = Interval(data_robot_[2]).inflate(0.1);
     (*state_vector)[2] &= heading;
     if ((*state_vector)[2].is_empty())
        (*state_vector)[2] = heading;
     Interval doppler = Interval(data_robot_[3]).inflate(0.1);
     (*state_vector)[3] &= doppler;
     if ((*state_vector)[3].is_empty())
        (*state_vector)[3] = doppler;
}

void IaSlam::contractPast(){
  for(int k = 0;k<5;k++)
{
  for (auto it =past.end() ; it != past.begin(); --it) 
  {
    if (it!=past.end()  && it!=past.end()-1 && past.size()>2 && it-1 !=past.begin())
    {
      (*temp_contract_vector).put(0,*((*it).first));
      (*temp_contract_vector).put(5,*((*(it+1)).first));
      (*temp_contract_vector).put(10,*u);
      updContract->contract(*temp_contract_vector);
      *((*it).first) = (*temp_contract_vector).subvector(0,4);
      *((*(it+1)).first) = (*temp_contract_vector).subvector(5,9);
      for (auto beaconMeas = (*it).second.begin(); beaconMeas != (*it).second.end();++beaconMeas)
      {
         std::map<int,std::vector<IntervalVector*> >::iterator itL;
         IntervalVector contract_vector(5);
         itL = landmarksMap.find((*beaconMeas).second);
         if (itL == landmarksMap.end())
            landmarksMap[(*beaconMeas).second] = std::vector<IntervalVector*>(1,new IntervalVector(2,Interval(-50,50)));
         (contract_vector)[0] = (*((*(it+1)).first))[0];
         (contract_vector)[1] = (*((*(it+1)).first))[1];
         (contract_vector)[2] = (*(landmarksMap[(*beaconMeas).second][0]))[0];
         (contract_vector)[3] = (*(landmarksMap[(*beaconMeas).second][0]))[1];
         (contract_vector)[4] = *((*beaconMeas).first);
         distContract->contract(contract_vector);
         (*((*(it+1)).first))[0] = (contract_vector)[0];
         (*((*(it+1)).first))[1] = (contract_vector)[1];
         if ((*(landmarksMap[(*beaconMeas).second][0])).max_diam()>0.5)
         {
           (*(landmarksMap[(*beaconMeas).second][0]))[0] = contract_vector[2];
           (*(landmarksMap[(*beaconMeas).second][0]))[1] = contract_vector[3];
         }
         *((*beaconMeas).first) = contract_vector[4];
      }// end for 
   }//end if
  }//end for
 

  for (auto it =past.begin() ; it != past.end(); ++it) 
  {
    if (it+1!=past.end())
    {
      (*temp_contract_vector).put(0,*((*it).first));
      (*temp_contract_vector).put(5,*((*(it+1)).first));
      (*temp_contract_vector).put(10,*u);
      updContract->contract(*temp_contract_vector);
      *((*it).first) = (*temp_contract_vector).subvector(0,4);
      *((*(it+1)).first) = (*temp_contract_vector).subvector(5,9);
      for (auto beaconMeas = (*it).second.begin(); beaconMeas != (*it).second.end();++beaconMeas)
      {
         std::map<int,std::vector< IntervalVector*> >::iterator itL;
         IntervalVector contract_vector(5);
         itL = landmarksMap.find((*beaconMeas).second);
         if (itL == landmarksMap.end())
            landmarksMap[(*beaconMeas).second] = std::vector<IntervalVector*>(1,new IntervalVector(2,Interval(-50,50)));
         (contract_vector)[0] = (*((*(it+1)).first))[0];
         (contract_vector)[1] = (*((*(it+1)).first))[1];
         (contract_vector)[2] = (*(landmarksMap[(*beaconMeas).second][0]))[0];
         (contract_vector)[3] = (*(landmarksMap[(*beaconMeas).second][0]))[1];
         (contract_vector)[4] = *((*beaconMeas).first);
         distContract->contract(contract_vector);
         (*((*(it+1)).first))[0] = (contract_vector)[0];
         (*((*(it+1)).first))[1] = (contract_vector)[1];
         if ((*(landmarksMap[(*beaconMeas).second][0])).max_diam()>0.5)
         {
           (*(landmarksMap[(*beaconMeas).second][0]))[0] = contract_vector[2];
           (*(landmarksMap[(*beaconMeas).second][0]))[1] = contract_vector[3];
         }
         *((*beaconMeas).first) = contract_vector[4];
      }//end for  
    }//end if
  }//end for
}
  if (past.size()>1)
  {
    (*state_vector) = *(past[past.size()-2].first);
    updateState();
  }
}

>>>>>>> 23c9ff033674454510a912db6b338eb85e326248
}//end namespace

using namespace ia_slam;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"ia_slam");
    
    IaSlam node;

    node.spin();

    return 0;
}
