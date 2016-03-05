#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{
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
           std::vector<IntervalVector> baseIntTmp = std::vector<IntervalVector>((landmarksMap[msg.id]).size(),IntervalVector(2,Interval::EMPTY_SET));
           std::vector<IntervalVector> otherIntTmp = std::vector<IntervalVector>((landmarksMap[(*it).id]).size(),IntervalVector(2,Interval::EMPTY_SET));
           int i,j;
           i=0;
           for (auto baseInt = landmarksMap[msg.id].begin();baseInt!=landmarksMap[msg.id].end();baseInt++)
           {
               j=0;
               for (auto otherInt = landmarksMap[(*it).id].begin();otherInt!=landmarksMap[(*it).id].end();otherInt++)
               {
                  (contract_vector)[0] = (**baseInt)[0];
                  (contract_vector)[1] = (**baseInt)[1];
                  (contract_vector)[2] = (**otherInt)[0];
                  (contract_vector)[3] = (**otherInt)[1];
                  (contract_vector)[4] = Interval((*it).dist).inflate(0.2);
                  distContract->contract(contract_vector);
                  baseIntTmp[i][0] |= contract_vector[0];
                  baseIntTmp[i][1] |= contract_vector[1];
                  otherIntTmp[j][0] |= contract_vector[2];
                  otherIntTmp[j][1] |= contract_vector[3];
                  j++;
               }
               i++;
           }
           std::vector<IntervalVector*> baseIntToRemove;
           std::vector<IntervalVector*> otherIntToRemove;
           for (i = 0;i<baseIntTmp.size();i++)
           {
               if (baseIntTmp[i].is_empty())
               {
                  baseIntToRemove.push_back(landmarksMap[msg.id][i]);
               }
               else
                  (*(landmarksMap[msg.id][i])).put(0,baseIntTmp[i]);
           }
           for (auto ist = baseIntToRemove.cbegin();ist!=baseIntToRemove.end();ist++)
           {
              auto position = std::find(landmarksMap[msg.id].begin(), landmarksMap[msg.id].end(), *ist);
              if (position != landmarksMap[msg.id].end())
              {
                 delete *position;
                 landmarksMap[msg.id].erase(position);
              }
           }
           for (i = 0;i<otherIntTmp.size();i++)
           {
               if (otherIntTmp[i].is_empty())
               {
                  otherIntToRemove.push_back(landmarksMap[(*it).id][i]);
               }
               else
                  (*(landmarksMap[(*it).id][i])).put(0,otherIntTmp[i]);
           }
           for (auto ist = otherIntToRemove.cbegin();ist!=otherIntToRemove.end();ist++)
           {
              auto position = std::find(landmarksMap[(*it).id].begin(), landmarksMap[(*it).id].end(), *ist);
              if (position != landmarksMap[(*it).id].end())
              {
                 delete *position;
                 landmarksMap[(*it).id].erase(position);
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
         std::vector<IntervalVector* > beacIntToRemove;
         IntervalVector newX(2,Interval::EMPTY_SET);
         if (landmarksMap[(*beaconMeas).second].size() == 1){
            (contract_vector)[0] = (*((*it).first))[0];
            (contract_vector)[1] = (*((*it).first))[1];
            (contract_vector)[2] = (*landmarksMap[(*beaconMeas).second][0])[0];
            (contract_vector)[3] = (*landmarksMap[(*beaconMeas).second][0])[1];
            (contract_vector)[4] = *((*beaconMeas).first);
            distSIVIA(landmarksMap[(*beaconMeas).second],contract_vector,5);
            landmarksMap[(*beaconMeas).second].erase(landmarksMap[(*beaconMeas).second].begin());
         }
         for (auto beacInt = landmarksMap[(*beaconMeas).second].begin();beacInt!=landmarksMap[(*beaconMeas).second].end();beacInt++)
         { 
            if ((**beacInt).is_empty())
               beacIntToRemove.push_back(*beacInt);
            else
            {
               (contract_vector)[0] = (*((*it).first))[0];
               (contract_vector)[1] = (*((*it).first))[1];
               (contract_vector)[2] = (**beacInt)[0];
               (contract_vector)[3] = (**beacInt)[1];
               (contract_vector)[4] = *((*beaconMeas).first);
               distContract->contract(contract_vector);
               newX[0] |= (contract_vector)[0];
               newX[1] |= (contract_vector)[1];
               if ((**beacInt).max_diam()>0.5)
               {
                 (**beacInt)[0] = contract_vector[2];
                 (**beacInt)[1] = contract_vector[3];
               }
               *((*beaconMeas).first) = contract_vector[4];
               if ((**beacInt).is_empty())
                  beacIntToRemove.push_back(*beacInt);
             }//end if
          }//end for
          (*((*it).first)).put(0,newX);
          for (auto ist = beacIntToRemove.cbegin();ist!=beacIntToRemove.end();++ist)
          {
              auto position = std::find(landmarksMap[(*beaconMeas).second].begin(), landmarksMap[(*beaconMeas).second].end(), *ist);
              if (position != landmarksMap[(*beaconMeas).second].end())
              {
                 delete *position;
                 landmarksMap[(*beaconMeas).second].erase(position);
              }
          }
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
         std::vector<IntervalVector* > beacIntToRemove;
         IntervalVector newX(2,Interval::EMPTY_SET);
         for (auto beacInt = landmarksMap[(*beaconMeas).second].begin();beacInt!=landmarksMap[(*beaconMeas).second].end();beacInt++)
         { 
            if ((**beacInt).is_empty())
               beacIntToRemove.push_back(*beacInt);
            else
            {
               (contract_vector)[0] = (*((*it).first))[0];
               (contract_vector)[1] = (*((*it).first))[1];
               (contract_vector)[2] = (**beacInt)[0];
               (contract_vector)[3] = (**beacInt)[1];
               (contract_vector)[4] = *((*beaconMeas).first);
               distContract->contract(contract_vector);
               newX[0] |= (contract_vector)[0];
               newX[1] |= (contract_vector)[1];
               if ((**beacInt).max_diam()>0.5)
               {
                 (**beacInt)[0] = contract_vector[2];
                 (**beacInt)[1] = contract_vector[3];
               }
               *((*beaconMeas).first) = contract_vector[4];
             }//end if
          }//end for
          (*((*it).first)).put(0,newX);
          for (auto ist = beacIntToRemove.cbegin();ist!=beacIntToRemove.end();ist++)
          {
              auto position = std::find(landmarksMap[(*beaconMeas).second].begin(), landmarksMap[(*beaconMeas).second].end(), *ist);
              if (position != landmarksMap[(*beaconMeas).second].end())
              {
                 delete *position;
                 landmarksMap[(*beaconMeas).second].erase(position);
              }
          }
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

}//end namespace
