#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{

void IaSlam::beaconDist(const ia_msgs::BeaconDist msg)
{
  //ROS_INFO("delay : %f",(msg.header.stamp-ros::Time::now()).toSec());
  if (quickstart_)
     start = true;
  else if (!start)
     return;
  map[msg.id] = Interval(msg.distance).inflate(sensorPrecision_);
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
                  distCut->contract(contract_vector);
                  baseIntTmp[i][0] |= contract_vector[0];
                  baseIntTmp[i][1] |= contract_vector[1];
                  otherIntTmp[j][0] |= contract_vector[2];
                  otherIntTmp[j][1] |= contract_vector[3];
                  ++j;
               }
              ++i;
           }
////////////////////////:managing empty set ////////////////////////
         //////////////////////////////////////////////////////
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
           if (baseIntToRemove.size() != landmarksMap[msg.id].size()){
              for (auto ist = baseIntToRemove.cbegin();ist!=baseIntToRemove.end();ist++)
              {
                 auto position = std::find(landmarksMap[msg.id].begin(), landmarksMap[msg.id].end(), *ist);
                 if (position != landmarksMap[msg.id].end())
                 {
                    delete *position;
                    landmarksMap[msg.id].erase(position);
                 }
              }
           }
           else{
             IntervalVector newX(2,Interval::EMPTY_SET);
             for (auto ist = landmarksMap[msg.id].cbegin();ist!=landmarksMap[msg.id].cend();ist++)
             {
                 newX |= *(*ist);
                 delete *ist;
             }
             landmarksMap[msg.id].clear();
             landmarksMap[msg.id].push_back(new IntervalVector(newX));
           }
           
     //////////////////////////////////////////////////////////     
           for (i = 0;i<otherIntTmp.size();i++)
           {
               if (otherIntTmp[i].is_empty())
               {
                  otherIntToRemove.push_back(landmarksMap[(*it).id][i]);
               }
               else
                  (*(landmarksMap[(*it).id][i])).put(0,otherIntTmp[i]);
           }

           if (otherIntToRemove.size() != landmarksMap[(*it).id].size()){// if we don't have to erase all box we proceed to delete the empty one
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
           else{ //else we make the union of all boxes to restore a bigger box (because it means we had over reduce the boxes)
             IntervalVector newX(2,Interval::EMPTY_SET);
             for (auto ist = landmarksMap[(*it).id].cbegin();ist!=landmarksMap[(*it).id].cend();ist++)
             {
                 newX |= *(*ist);
                 delete *ist;
             }
             landmarksMap[(*it).id].clear();
             landmarksMap[(*it).id].push_back(new IntervalVector(newX));
           }

///////////////////////////////////////////////////////////////////////////
        }
     }
  }
}

void IaSlam::betweenRobot(const ia_msgs::StampedInterval msg){
  ROS_INFO("received msg from other %f",(*msg.data.cbegin()).data[1].width);
  std::map<int,std::vector< IntervalVector*> >::iterator itL;
  IntervalVector contract_vector(5);

  IntervalVector other(2);
  other[0] = Interval((*msg.data.cbegin()).data[0].position.x,(*msg.data.cbegin()).data[0].position.x+(*msg.data.cbegin()).data[0].width);
  other[1] = Interval((*msg.data.cbegin()).data[0].position.y,(*msg.data.cbegin()).data[0].position.y+(*msg.data.cbegin()).data[0].height);
  (contract_vector)[0] = (*state_vector)[0];
  (contract_vector)[1] = (*state_vector)[1];
  (contract_vector)[2] = other[0];
  (contract_vector)[3] = other[1];
  (contract_vector)[4] = Interval((*msg.data.cbegin()).data[1].width);//.inflate(sensorPrecision_);
  distCut->contract(contract_vector);
  (*state_vector)[0] = (contract_vector)[0];
  (*state_vector)[1] = (contract_vector)[1];
  //update pose of other robot
  std::map<int,IntervalVector* >::iterator itOther;
  itOther = otherRobotMap.find((int) (*msg.data.cbegin()).data[1].height);
  if (itOther != otherRobotMap.end())
  {
     (*(otherRobotMap[(int) (*msg.data.cbegin()).data[1].height])).put(0,other); 
  }
  else{
     otherRobotMap[(int) (*msg.data.cbegin()).data[1].height] = new IntervalVector(other);
  }

  for (auto landIt=msg.data.cbegin()+1; landIt!=msg.data.cend();++landIt){
     //convert to interval
     std::vector< IntervalVector*> newBoxes;
     msgToBoxes(newBoxes,(*landIt).data);
     itL = landmarksMap.find((*landIt).id);
     if (itL != landmarksMap.end())
     {
        std::vector<IntervalVector> landIntTmp = std::vector<IntervalVector>((landmarksMap[(*landIt).id]).size(),IntervalVector(2,Interval::EMPTY_SET));
        for (int i = 0; i < landmarksMap[(*landIt).id].size(); ++i)
        {
           std::vector<IntervalVector> tmpBox = std::vector<IntervalVector>(newBoxes.size(),IntervalVector(2,Interval::EMPTY_SET));
           for (int j = 0;j<newBoxes.size();++j)
           {
               tmpBox[j] = (*(landmarksMap[(*landIt).id][i])) & (*newBoxes[j]);
           }
           for (auto tmpBoxIt =tmpBox.cbegin();tmpBoxIt!=tmpBox.cend();++tmpBoxIt)
           {
               landIntTmp[i] |= *tmpBoxIt; 
           }
        }
   ////////////////////////:managing empty set ////////////////////////
        std::vector<IntervalVector*> baseIntToRemove;
        for (int i = 0;i<landmarksMap[(*landIt).id].size();i++)
        {
             if (landIntTmp[i].is_empty())
                baseIntToRemove.push_back(landmarksMap[(*landIt).id][i]);
             else
                (*(landmarksMap[(*landIt).id][i])).put(0,landIntTmp[i]);
        }
        if (baseIntToRemove.size() != landmarksMap[(*landIt).id].size()){
            for (auto ist = baseIntToRemove.cbegin();ist!=baseIntToRemove.cend();++ist)
            {
               auto position = std::find(landmarksMap[(*landIt).id].begin(), landmarksMap[(*landIt).id].end(), *ist);
               if (position != landmarksMap[(*landIt).id].end())
               {
                  delete *position;
                  landmarksMap[(*landIt).id].erase(position);
               }
             }
        }
        for (auto ist = newBoxes.begin();ist!=newBoxes.end();++ist)
        {
                  delete *ist;
        }
        newBoxes.clear();
   ///////////////////////////////////////////////////////////////////////////
     } //end if
     else{
        landmarksMap[(*landIt).id] = newBoxes;
     }
  }//end for
}

void IaSlam::msgToBoxes(std::vector< IntervalVector*> &newBoxes,const std::vector<ia_msgs::Interv> data){
     for (auto it=data.cbegin();it!=data.cend();++it){
        IntervalVector* newVect = new IntervalVector(2);
        (*newVect)[0] = Interval((*it).position.x,(*it).position.x+(*it).width);
        (*newVect)[1] = Interval((*it).position.y,(*it).position.y+(*it).height);
        newBoxes.push_back(newVect);
     }
}

void IaSlam::publishInterval(){// send interval boxes to rviz and/or other robots
   ia_msgs::Interval position_msg;
   ia_msgs::StampedInterval beacons_msg;
   position_msg.header.stamp =  ros::Time::now();
   beacons_msg.header.stamp =  ros::Time::now();
   position_msg.header.frame_id = map_frame_;
   beacons_msg.header.frame_id = map_frame_;
   //add pose to message
   ia_msgs::IdInterval selfPos;
   ia_msgs::Interv newPoint;
   ia_msgs::Interv blandPoint;
   newPoint.position.x = (*state_vector)[0].lb();
   newPoint.position.y = (*state_vector)[1].lb();
   newPoint.position.z = 0;
   newPoint.width = (*state_vector)[0].diam();
   newPoint.height = (*state_vector)[1].diam();
   blandPoint.height = id_robot_;
   position_msg.data.push_back(newPoint);
   selfPos.data.push_back(newPoint);
   selfPos.data.push_back(blandPoint);
   selfPos.id = 254;
   beacons_msg.data.push_back(selfPos);
   ////////////////////////////
   for (auto it = landmarksMap.cbegin(); it != landmarksMap.cend(); ++it)
       intervalToMsg(beacons_msg,(*it).first, (*it).second);
   
   beacon_pub_.publish(beacons_msg);
   position_pub_.publish(position_msg);
}

void IaSlam::intervalToMsg(ia_msgs::StampedInterval &interv,int i ,const std::vector<IntervalVector*> &boxes){
     ia_msgs::IdInterval newList;
     newList.id = i;
     for (auto box = boxes.cbegin();box!=boxes.cend();box++)
     { 
       ia_msgs::Interv newPoint;
       newPoint.position.x = (*(*box))[0].lb();
       newPoint.position.y = (*(*box))[1].lb();
       newPoint.position.z = 0;
       newPoint.width = (*(*box))[0].diam();
       newPoint.height = (*(*box))[1].diam();
       newList.data.push_back(newPoint);
     }
     interv.data.push_back(newList);
}

void IaSlam::internRobot(const std_msgs::Float64MultiArray msg)
{
  data_robot_.clear();
  for (auto it = msg.data.cbegin();it!=msg.data.cend();it++)
     data_robot_.push_back(*it);
}

void IaSlam::toControllerMsg(){
     ia_msgs::StampedInterval msg;
     msg.header.frame_id = map_frame_;
     ia_msgs::IdInterval newList;
     ia_msgs::Interv newPoint;
     newPoint.position.x = (*state_vector)[0].lb();
     newPoint.position.y = (*state_vector)[1].lb();
     newPoint.width = (*state_vector)[0].diam();
     newPoint.height = (*state_vector)[1].diam();
     newList.data.push_back(newPoint);
     newList.id = id_robot_;
     msg.data.push_back(newList);
     for (auto it=otherRobotMap.cbegin();it!=otherRobotMap.cend();++it){
         newPoint.position.x = (*(*it).second)[0].lb();
         newPoint.position.y = (*(*it).second)[1].lb();
         newPoint.width = (*(*it).second)[0].diam();
         newPoint.height = (*(*it).second)[1].diam();
         newList.data.push_back(newPoint);
         newList.id = (*it).first;
         msg.data.push_back(newList);
     }
     toController_pub_.publish(msg);
}

}//end namespace
