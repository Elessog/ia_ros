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

#include "ia_msgs/Interval.h"
#include "ia_msgs/BeaconDist.h"
#include "ia_msgs/Start_Slam.h"
#include <tf/transform_listener.h>

using namespace ibex;
namespace ia_slam
{

class IaSlam
{
 public:
   IaSlam();
   ~IaSlam();
   
   void spin();
 private:
   double k1_;
   double k2_;
   int ros_rate;
   int nb_outliers_;

   ros::NodeHandle nh;
   std::string map_frame_;
   std::string base_frame_;
   std::string beacon_topic_;
   
   ros::Subscriber beacon_sub_;

   ros::ServiceServer service_;
   ros::Publisher beacon_pub_;
   ros::Publisher position_pub_;
   double sensorPrecision_;
  
   bool starterControl(ia_msgs::Start_Slam::Request &req,ia_msgs::Start_Slam::Response &res);

   std::map<int,std::pair<Interval,bool> > map;
   std::map<int,std::pair<Interval,bool> > waitMap;
   std::map<int,IntervalVector* > landmarksMap;
   void beaconDist(const ia_msgs::BeaconDist msg);
   void ia_iter();
   void propageConstraint();
   void publishInterval();
   void intervalToMsg(ia_msgs::Interval &interv,const IntervalVector &box);
   void updateState();
   void distcontract(int id_beacon,Interval &dist);
   void contractPast();
/////////////////////////////////////
   bool start;
   bool is_prop_constraint_;
   IntervalVector *state_vector;
   IntervalVector *dstate_vector;
   IntervalVector *u;
   IntervalVector *temp_contract_vector; 
   std::vector<std::pair<IntervalVector*,std::vector<std::pair<Interval*,int> > > > past;
   CtcFixPoint *distContract;
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
   Function *distfunc;
   Function *updfunc;
   double dt;
};


}
