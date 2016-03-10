#include "ia_slam/ia_slam_node.h"

namespace ia_slam {
void IaSlam::distSIVIA(std::vector<IntervalVector*> &in,IntervalVector X,double eps=1){
   int i;
   std::stack<IntervalVector> s;
   s.push(X);
   IntervalVector tempVector(2);
   int j=in.size();
   int k = 0;
   while (!s.empty()) 
   {
       IntervalVector box=s.top();
       s.pop();
       distContract->contract(box);
       tempVector = (box.subvector(2,3));
       if (box.is_empty() )
       {
          continue;
       }
       else if (tempVector.max_diam()>eps)
       {
           i = tempVector.extr_diam_index(false)+2;
           std::pair<IntervalVector,IntervalVector> p=box.bisect(i);
           s.push(p.first);
           s.push(p.second);
           k++;
       }
       else //if (k < division_box_*division_box_) 
       {
           in.push_back(new IntervalVector(tempVector));
       }
       /*else
       {
         while (!s.empty())
        {
            in.push_back(new IntervalVector(s.top()));
            s.pop();
         }
       }  */  
   }

ROS_INFO("Sivia contraction %d",(int) in.size()-j);

}

} //end namespace
