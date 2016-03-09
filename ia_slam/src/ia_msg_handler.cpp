#include "ia_slam/ia_slam_node.h"

using namespace ibex;
namespace ia_slam 
{

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

           if (otherIntToRemove.size() != landmarksMap[(*it).id].size()){
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
           else{
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
  ROS_INFO("received msg from other");
  std::map<int,std::vector< IntervalVector*> >::iterator itL;
  IntervalVector contract_vector(5);
  for (auto landIt=msg.data.cbegin(); landIt!=msg.data.cend();++landIt){
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

}//end namespace
