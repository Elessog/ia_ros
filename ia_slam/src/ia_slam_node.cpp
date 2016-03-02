#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{

IaSlam::IaSlam():start(false),is_prop_constraint_(false)
{
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh_;

    if (!nh_private_.getParam ("map_frame", map_frame_))
        map_frame_ = "/map";
    if (!nh_private_.getParam ("robot_frame", base_frame_))
        base_frame_ = "/robot";
    if (!nh_private_.getParam ("beacon_topic", beacon_topic_))
        beacon_topic_= "beacon_echo";
    if (!nh_private_.getParam ("sensorPrecision", sensorPrecision_))
        sensorPrecision_ = 0.2;
    if (!nh_private_.getParam ("ros_rate", ros_rate))
        ros_rate = 30;
    if (!nh_private_.getParam ("nb_outliers", nb_outliers_))
        nb_outliers_ = 3;
    
    service_ = nh_private_.advertiseService("starter", &IaSlam::starterControl,this);
    beacon_sub_ = nh_.subscribe(beacon_topic_,1, &IaSlam::beaconDist,this);
    beacon_pub_ = nh_private_.advertise<ia_msgs::Interval>("beacons", 10);
    position_pub_ = nh_private_.advertise<ia_msgs::Interval>("pose", 10);
    state_vector = new IntervalVector(5,Interval(0,0));
    dstate_vector = new IntervalVector(5,Interval(-50,50));
    temp_contract_vector = new IntervalVector(12,Interval(-50,50));
    u = new IntervalVector(2,Interval(-5,5));
    x = new Variable();
    y = new Variable();
    ax = new Variable();
    ay = new Variable();
    d = new Variable();
    uX = new Variable(5);
    ud = new Variable(2);
    X = new Variable(5);
    distfunc= new Function(*x,*y,*ax,*ay,*d,sqr(*x-*ax)+sqr(*y-*ay)-sqr(*d));
    updfunc= new Function(*uX,*X,*ud,Return(
                                 (*X)[0] + (*state_vector)[3]*cos((*state_vector)[4])*cos((*state_vector)[2]) - (*uX)[0],
                                 (*X)[0] + (*state_vector)[3]*cos((*state_vector)[4])*sin((*state_vector)[2]) - (*uX)[1],
                                 (*X)[0] + (*state_vector)[3]*sin((*state_vector)[4])/3.0 - (*uX)[2],
                                 (*X)[0] + (*ud)[0] - (*uX)[3],
                                 (*X)[0] + (*ud)[1] - (*uX)[4]
                                 ));
    c =new CtcFwdBwd(*distfunc);
    distContract =new CtcFixPoint(*c,1e-01);
    s =new CtcFwdBwd(*updfunc);
    updContract =new CtcFixPoint(*s,1e-01);
    dt = 1/float(ros_rate);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    double roll, pitch, yaw;
    try{
      listener.waitForTransform(map_frame_, base_frame_,
                              ros::Time::now(), ros::Duration(3.0));
      listener.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform);
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      (*state_vector)[0] = Interval(transform.getOrigin().x()).inflate(1);
      (*state_vector)[1] = Interval(transform.getOrigin().y()).inflate(1);
      (*state_vector)[2] = Interval(yaw).inflate(0.1);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      exit(-1);
    }
}

IaSlam::~IaSlam(){} 

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
  ROS_INFO("received msg");
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
     publishInterval();
     
}

void IaSlam::propageConstraint(){
  is_prop_constraint_ = true;

  is_prop_constraint_ = false;
  map.swap(waitMap);
  waitMap.clear();
}

void IaSlam::publishInterval(){
   ia_msgs::Interval position_msg;
   ia_msgs::Interval beacons_msg;
   position_msg.header.stamp =  ros::Time::now();
   beacons_msg.header.stamp =  ros::Time::now();
   position_msg.header.frame_id = map_frame_;
   beacons_msg.header.frame_id = map_frame_;
   for (auto it = landmarksMap.cbegin(); it != landmarksMap.cend(); ++it)
       intervalToMsg(beacons_msg,*((*it).second));
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

void IaSlam::intervalToMsg(ia_msgs::Interval &interv,const IntervalVector &box){
     ia_msgs::Interv newPoint;
     newPoint.position.x = box[0].lb();
     newPoint.position.y = box[1].lb();
     newPoint.position.z = 0;
     newPoint.width = box[0].diam();
     newPoint.height = box[1].diam();
     interv.data.push_back(newPoint);
}

void IaSlam::updateState(){
     (*dstate_vector)[0] = (*state_vector)[3]*cos((*state_vector)[4])*cos((*state_vector)[2]);//x
     (*dstate_vector)[1] = (*state_vector)[3]*cos((*state_vector)[4])*sin((*state_vector)[2]);//y
     (*dstate_vector)[2] = (*state_vector)[3]*sin((*state_vector)[4])/3.0;//thetap
     (*dstate_vector)[3] = (*u)[0];//acc
     (*dstate_vector)[4] = (*u)[1];//deltap
     (*state_vector)[0] += (*dstate_vector)[0]*dt;
     (*state_vector)[1] += (*dstate_vector)[1]*dt;
     (*state_vector)[2] += (*dstate_vector)[2]*dt;
     (*state_vector)[3] += (*dstate_vector)[3]*dt;
     (*state_vector)[4] += (*dstate_vector)[4]*dt;
}    

void IaSlam::distcontract(int id_beacon,Interval &dist){
     /*std::map<int,IntervalVector*>::iterator it;
     IntervalVector contract_vector(5);
     it = landmarksMap.find(id_beacon);
     if (it == landmarksMap.end())
        landmarksMap[id_beacon] =new IntervalVector(2,Interval(-50,50));
     (*temp_contract_vector)[0] = (*state_vector)[0];
     (*temp_contract_vector)[1] = (*state_vector)[1];
     (*temp_contract_vector)[2] = (*landmarksMap[id_beacon])[0];
     (*temp_contract_vector)[3] = (*landmarksMap[id_beacon])[1];
     (*temp_contract_vector)[4] = dist;
     std::cout<<(*temp_contract_vector)<<std::endl;
     distContract->contract(contract_vector);
     (*state_vector)[0] = (*temp_contract_vector)[0];
     (*state_vector)[1] = (*temp_contract_vector)[1];
     (*landmarksMap[id_beacon])[0] = (*temp_contract_vector)[2];
     (*landmarksMap[id_beacon])[1] = (*temp_contract_vector)[3];
     dist = (*temp_contract_vector)[4];*/
}

void IaSlam::contractPast(){
  for (auto it =past.begin() ; it != past.end(); ++it) {
    (*temp_contract_vector).put(0,*((*it).first));
    (*temp_contract_vector).put(5,*((*(it+1)).first));
    (*temp_contract_vector).put(10,*u);
    updContract->contract(*temp_contract_vector);
    *((*it).first) = (*temp_contract_vector).subvector(0,4);
    *((*(it+1)).first) = (*temp_contract_vector).subvector(5,9);
    for (auto iit = (*it).second.begin(); iit != (*it).second.end();++iit){
       std::map<int,IntervalVector*>::iterator itL;
       IntervalVector contract_vector(5);
       itL = landmarksMap.find((*iit).first);
       if (itL == landmarksMap.end())
          landmarksMap[(*iit).first] =new IntervalVector(2,Interval(-50,50));
       (contract_vector)[0] = (*((*(it+1)).first))[0];
       (contract_vector)[1] = (*((*(it+1)).first))[1];
       (contract_vector)[2] = (*landmarksMap[id_beacon])[0];
       (contract_vector)[3] = (*landmarksMap[id_beacon])[1];
       (contract_vector)[4] = *((*iit).second);
       distContract->contract(contract_vector);
       (*((*(it+1)).first))[0] = (contract_vector)[0];
       (*((*(it+1)).first))[1] = (contract_vector)[1];
       landmarksMap[id_beacon][0] = contract_vector[2];
       landmarksMap[id_beacon][1] = contract_vector[3];
       *((*iit).second) = contract_vector[4];
    }
    /*  // Get a copy of the domain of x[t]
      IntervalVector xt=x.subvector(2*t,2*t+1);
      // Set the time
      scan.set_time(t);
      // Contract with the scanning
      scan.contract(xt);
      // Update the box "x" with the new domain for x[t]
      x.put(2*t,xt);
      if (t<T-1) {
        // Get a copy of the domain of x[t] and x[t+1]
         IntervalVector xtt1=x.subvector(2*t,2*(t+1)+1);
         // Set the time
         speed.set_time(t);
         // Contract with the speed vector
         speed.contract(xtt1);
         // Update the box
         x.put(2*t,xtt1);
     }*/
  }
}

}//end namespace

using namespace ia_slam;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"ia_slam");
    
    IaSlam node;

    node.spin();

    return 0;
}
