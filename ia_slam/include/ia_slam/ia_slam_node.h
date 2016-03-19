// ros stuff
#include <ros/ros.h>

#include "ibex_IntervalVector.h"
#include <ibex_Ctc.h>
#include <ibex_CtcFwdBwd.h>
#include <ibex_CtcFixPoint.h>
#include <ibex_CtcUnion.h>
#include <ibex_CtcCompo.h>
#include <ibex_CtcNotIn.h>
#include <ibex_Function.h>
#include <ibex_CtcInverse.h>
#include <ibex_CtcPolar.h>
#include <ibex_CtcSegment.h>
#include <ibex_CtcQInter.h>
#include <ibex_CtcPixelMap.h>
#include "ibex_Ctc3BCid.h"

#include "ia_msgs/StampedInterval.h"
#include "ia_msgs/Interval.h"
#include "ia_msgs/BeaconDist.h"
#include "ia_msgs/Start_Slam.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf/transform_listener.h>

#include <fstream>
#include <iostream>

using namespace ibex;
typedef std::vector<std::pair<Interval*,int> > it_beac;
typedef std::vector<std::pair<IntervalVector*,it_beac > > past_vector;


namespace ia_slam
{

class IaSlam
{
 public:
   IaSlam();
   ~IaSlam();
   
   void init();
   void spin();
 private:
   double k1_;
   double k2_;
   int ros_rate;
   int nb_outliers_;
   int max_past_iter_;
   int id_robot_;
   bool quickstart_;

   ros::NodeHandle nh;
   std::string map_frame_;
   std::string base_frame_;
   std::string beacon_topic_;
   std::string internal_topic_;
   std::string external_interv_topic_;

   ros::Subscriber beacon_sub_;
   ros::Subscriber internal_sub_;
   ros::Subscriber external_interv_sub_;

   ros::ServiceServer service_;
   ros::Publisher beacon_pub_;
   ros::Publisher position_pub_;
   ros::Publisher toController_pub_;
   ros::Time lastIter;
   ros::Time contractTime;
   double sensorPrecision_;
   double gps_precision_;
   double division_box_;
   double heading_precision_;
   double speed_precision_;
   double headingWheel_precision_;
  
   bool starterControl(ia_msgs::Start_Slam::Request &req,ia_msgs::Start_Slam::Response &res);

   std::map<int,Interval > map;
   std::map<int,IntervalVector*> otherRobotMap;
   std::map<int,std::vector< IntervalVector*> > landmarksMap;
   void beaconDist(const ia_msgs::BeaconDist msg);
   void internRobot(const std_msgs::Float64MultiArray msg);
   void betweenRobot(const ia_msgs::StampedInterval msg);
   void ia_iter();
   void publishInterval();
   void intervalToMsg(ia_msgs::StampedInterval &interv,int i, const std::vector<IntervalVector*> &boxes);
   void msgToBoxes(std::vector< IntervalVector*> &newBoxes,const std::vector<ia_msgs::Interv> data);
   void toControllerMsg();
   void updateState(double dtt,bool b);
   void updateOtherRobot(double dt);
   void contractPast();
   void presentToPast();
   void pastToPresent();
   void distSIVIA(std::vector<IntervalVector*> &in,IntervalVector X,double eps);
   void contractIterDist(it_beac::iterator &beaconMeas,past_vector::iterator &it);
   void dump();
/////////////////////////////////////
   bool start;
   std::vector<double> data_robot_;
   IntervalVector *state_vector;
   IntervalVector *dstate_vector;
   IntervalVector *u;
   IntervalVector *temp_contract_vector; 
   past_vector past;
   std::vector<double> pastDt;
   CtcFixPoint *distContract;
   Ctc3BCid *distCut;
   CtcFwdBwd *c;
   CtcFixPoint *updContract;
   CtcFwdBwd *s;
   CtcCompo *compo;
   Variable *x;
   Variable *y;
   Variable *ax;
   Variable *ay;
   Variable *d;
   Variable *uX;
   Variable *ud;
   Variable *X;
   Variable *idt;
   Function *distfunc;
   Function *updfunc;
   double dt;
};


}
