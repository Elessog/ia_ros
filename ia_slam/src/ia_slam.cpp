#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{

void IaSlam::ia_iter(){
     ros::Time nowT  = ros::Time::now();
     
     updateState((nowT-lastIter).toSec(),true);
     lastIter = nowT;
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

void IaSlam::updateState(double dtt,bool b = true){
     if (b)
        pastDt.push_back(dtt);
     (*dstate_vector)[3] = (*u)[0]&(Interval(data_robot_[0]).inflate(0.1));//acc
     (*dstate_vector)[4] = (*u)[1]&(Interval(data_robot_[1]).inflate(0.1));//deltap

      
     (*state_vector)[3] += (*dstate_vector)[3]*dtt;
     
     (*state_vector)[4] += (*dstate_vector)[4]*dtt;
     Interval headingWheel = Interval(data_robot_[4]).inflate(headingWheel_precision_);
     (*state_vector)[4] &= headingWheel;
     if ((*state_vector)[4].is_empty())
        (*state_vector)[4] = headingWheel;

     (*dstate_vector)[2] = (*state_vector)[3]*sin((*state_vector)[4])/3.0;//thetap
     (*state_vector)[2] += (*dstate_vector)[2]*dtt;

     Interval heading = Interval(data_robot_[2]).inflate(heading_precision_);
     (*state_vector)[2] &= heading;
     if ((*state_vector)[2].is_empty())
        (*state_vector)[2] = heading;
     Interval doppler = Interval(data_robot_[3]).inflate(speed_precision_);
     (*state_vector)[3] &= doppler;
     if ((*state_vector)[3].is_empty())
        (*state_vector)[3] = doppler;
     

     (*dstate_vector)[0] = (*state_vector)[3]*cos((*state_vector)[4])*cos((*state_vector)[2]);//x
     (*dstate_vector)[1] = (*state_vector)[3]*cos((*state_vector)[4])*sin((*state_vector)[2]);//y
     
     (*state_vector)[0] += (*dstate_vector)[0]*dtt;
     (*state_vector)[1] += (*dstate_vector)[1]*dtt;


}

void IaSlam::contractPast(){
  for (int k = 0;k<1;k++)
  {
     if (past.size()>250){
        for (auto it=(*(past.begin())).second.begin();it!=(*(past.begin())).second.end();++it)     
            delete (*it).first;
        delete (*(past.begin())).first;
        past.erase(past.begin());
        pastDt.erase(pastDt.begin());
     }
     presentToPast();
     pastToPresent();
  }
  if (past.size()>1)
  {
    (*state_vector) = *(past[past.size()-1].first);
  }
}

void IaSlam::presentToPast(){
  int idx = past.size();
  for (auto it =past.end()-1 ; it != past.begin(); --it) //go through time in reverse
  {
    if (it!=past.end()  && past.size()>2)
    {
      ///////// contract over state equation /////////
      if (it!=past.end()-1){
         (*temp_contract_vector).put(0,*((*it).first));
         (*temp_contract_vector).put(5,*((*(it+1)).first));
         (*temp_contract_vector).put(10,*u);
         (*temp_contract_vector)[12]=Interval(pastDt[idx+1]).inflate(0.02);
         updContract->contract(*temp_contract_vector);
         if (!(*temp_contract_vector).subvector(0,1).is_empty() && !(*temp_contract_vector).subvector(5,6).is_empty()) {
               *((*it).first) = (*temp_contract_vector).subvector(0,4);
               *((*(it+1)).first) = (*temp_contract_vector).subvector(5,9);
         }
      }
      //////////////////distance to landmarks /////////////////
      for (auto beaconMeas = (*it).second.begin(); beaconMeas != (*it).second.end();++beaconMeas)
      {
         contractIterDist(beaconMeas,it);
      }// end for 
   }//end if
  }//end for
  idx--;
}

void IaSlam::pastToPresent(){
  int idx = 0;
  for (auto it =past.begin(); it != past.end(); ++it) //go through time from init to present
  {
    if (it+1!=past.end())
    {
      ///////// contract over state equation /////////
      /*(*temp_contract_vector).put(0,*((*it).first));
      (*temp_contract_vector).put(5,*((*(it+1)).first));
      (*temp_contract_vector).put(10,*u);
      (*temp_contract_vector)[12]=Interval(pastDt[idx]);
      updContract->contract(*temp_contract_vector);
      if (!(*temp_contract_vector).subvector(0,1).is_empty() && !(*temp_contract_vector).subvector(5,6).is_empty()) {
            *((*it).first) = (*temp_contract_vector).subvector(0,4);
            *((*(it+1)).first) = (*temp_contract_vector).subvector(5,9);
      }*/
      //////////////////distance to landmarks /////////////////
      for (auto beaconMeas = (*it).second.begin(); beaconMeas != (*it).second.end();++beaconMeas)
      {
         contractIterDist(beaconMeas,it);
      }//end for  
    }//end if
    idx++;
  }//end for
}


void IaSlam::contractIterDist(it_beac::iterator &beaconMeas,past_vector::iterator &it){
// contract with distances constraint for the time at iteration it
   std::map<int,std::vector<IntervalVector*> >::iterator itL;
   IntervalVector contract_vector(5);
   itL = landmarksMap.find((*beaconMeas).second);
   if (itL == landmarksMap.end())
      landmarksMap[(*beaconMeas).second] = std::vector<IntervalVector*>(1,new IntervalVector(2,Interval(-50,50)));
   std::vector<IntervalVector* > beacIntToRemove;
   IntervalVector newX(2,Interval::EMPTY_SET);
   if (landmarksMap[(*beaconMeas).second].size() < 4)
   {
       int k=0;
       double surface;
       for (int i=0;i<landmarksMap[(*beaconMeas).second].size();i++){
          if ( surface < (*landmarksMap[(*beaconMeas).second][0])[0].diam() * ((*landmarksMap[(*beaconMeas).second][0])[1].diam()) ){
             k=i;
             surface = (*landmarksMap[(*beaconMeas).second][0])[0].diam() * ((*landmarksMap[(*beaconMeas).second][0])[1].diam());
          }
       }
       (contract_vector)[0] = (*((*it).first))[0];
       (contract_vector)[1] = (*((*it).first))[1];
       (contract_vector)[2] = (*landmarksMap[(*beaconMeas).second][k])[0];
       (contract_vector)[3] = (*landmarksMap[(*beaconMeas).second][k])[1];
       (contract_vector)[4] = *((*beaconMeas).first);
       if ((*landmarksMap[(*beaconMeas).second][k]).max_diam() > 0.5) //if the biggest box is big enough we pave it
       {
          distSIVIA(landmarksMap[(*beaconMeas).second],contract_vector,(*landmarksMap[(*beaconMeas).second][k]).max_diam()/division_box_);
          landmarksMap[(*beaconMeas).second].erase(landmarksMap[(*beaconMeas).second].begin()+k);
       }
   }
   if (landmarksMap[(*beaconMeas).second].size()==0){
      dump();
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
          if (landmarksMap[(*beaconMeas).second].size()==1) {
             if  ((**beacInt).max_diam()>0.5)
             {
               (**beacInt)[0] = contract_vector[2];
               (**beacInt)[1] = contract_vector[3];
             }
          }
          else{
             (**beacInt)[0] = contract_vector[2];
             (**beacInt)[1] = contract_vector[3];
          }
          //*((*beaconMeas).first) = contract_vector[4];
          if ((**beacInt).is_empty())//if a box from the landmark is empty we put it in the to remove list
             beacIntToRemove.push_back(*beacInt);
       }//end if
    }//end for
    if (!(contract_vector).subvector(0,1).is_empty() ) {// if we have over contracted the pose box we restore the last box
       (*((*it).first)).put(0,newX);
    }
    if (beacIntToRemove.size() < landmarksMap[(*beaconMeas).second].size()){//check if we over contracted the landmark
       for (auto ist = beacIntToRemove.cbegin();ist!=beacIntToRemove.end();++ist)//delete empty boxes
       {
          auto position = std::find(landmarksMap[(*beaconMeas).second].begin(), landmarksMap[(*beaconMeas).second].end(), *ist);
          if (position != landmarksMap[(*beaconMeas).second].end())
          {
             delete *position;
             landmarksMap[(*beaconMeas).second].erase(position);
          }
       }
    }
    else{
       newX[0] = Interval::EMPTY_SET;
       newX[1] = Interval::EMPTY_SET;
       for (auto ist = landmarksMap[(*beaconMeas).second].cbegin();ist!=landmarksMap[(*beaconMeas).second].cend();ist++)
       {
          newX |= *(*ist);
          delete *ist;
       }
       landmarksMap[(*beaconMeas).second].clear();
       landmarksMap[(*beaconMeas).second].push_back(new IntervalVector(newX));
    }
}

}//end namespace
