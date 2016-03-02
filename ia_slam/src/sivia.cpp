#include "ia_slam/sivia.hpp"



void clSIVIA::setRecord(std::string const & msg,float fps,bool rectangle = false){
   record = cv::VideoWriter(msg,CV_FOURCC('D','I','V','X'), fps,cv::Size(width_,height_), true);
   is_rect = rectangle;
   is_recording = true;
}

void clSIVIA::startScreening(bool rectangle){
     is_rect = rectangle;
     is_screening = true;
     mutex = PTHREAD_MUTEX_INITIALIZER;
     if (pthread_create(&thread, NULL,&clSIVIA::screening_helper,this) != 0)
             exit(EXIT_FAILURE);
}

void clSIVIA::screening(){
     show = true;
     while(show){
        pthread_mutex_lock(&mutex);
        if (change)
          imageColor1.copyTo(imageColor2);
        change = false;
        pthread_mutex_unlock(&mutex);
        cv::imshow("Boxes",imageColor2);
        cv::waitKey(20);
     }
}

void clSIVIA::stopScreening(){
     show = false;
}

//Method which create the pavage with SIVIA:
int clSIVIA::testIm(const float echellePixel,const IntervalVector& X,const int i0,const int j0){
        int resp[2];
        clSIVIA::computeBox(echellePixel,resp,X,i0,j0);

        if (resp[0] == 0)
            return IBOOL::OUT;
        if (resp[0] == resp[1])
            return IBOOL::IN;
        else
            return IBOOL::UNK;
}

void clSIVIA::drawBox(const IntervalVector& X,const float echellePixel,const int i0,const int j0){
     int A[2];
     toPixels(echellePixel,A,X[0].lb(),X[1].ub(),i0,j0);
     int C[2];
     toPixels(echellePixel,C,X[0].ub(),X[1].lb(),i0,j0);
     std::vector<cv::Point> fillContSingle;
     fillContSingle.push_back(cv::Point(A[0],A[1]));
     fillContSingle.push_back(cv::Point(C[0],A[1]));
     fillContSingle.push_back(cv::Point(C[0],C[1]));
     fillContSingle.push_back(cv::Point(A[0],C[1]));
     std::vector<std::vector<cv::Point> > fillContAll;
     fillContAll.push_back(fillContSingle);
     cv::fillPoly( past, fillContAll, cv::Scalar(1,1,1));
}

void clSIVIA::drawReal(const IntervalVector& X,const float echellePixel,const int i0,const int j0,cv::Scalar color){
     int A[2];
     toPixels(echellePixel,A,X[0].lb(),X[1].ub(),i0,j0);
     int C[2];
     toPixels(echellePixel,C,X[0].ub(),X[1].lb(),i0,j0);
     std::vector<cv::Point> fillContSingle;
     fillContSingle.push_back(cv::Point(A[0],A[1]));
     fillContSingle.push_back(cv::Point(C[0],A[1]));
     fillContSingle.push_back(cv::Point(C[0],C[1]));
     fillContSingle.push_back(cv::Point(A[0],C[1]));
     std::vector<std::vector<cv::Point> > fillContAll;
     fillContAll.push_back(fillContSingle);
     cv::fillPoly( imageColor, fillContAll, color);
     if (is_rect)
       cv::rectangle(imageColor,cv::Point(A[0],A[1]),cv::Point(C[0],C[1]), cv::Scalar(0,0,0));
}

void clSIVIA::imSIVIA(const Separator X,const py::list& lst,double rang2,const float echellePixel,
                 const int i0,const int j0,double eps=0.1,bool efficient=true){
   std::vector<IntervalVector> in;
   std::vector<IntervalVector> out;
   std::vector<IntervalVector> maybe;
   std::vector<IntervalVector> border;
   std::stack<IntervalVector> s;
   s.push(X);
   std::cout << "Initialisation Finished" << std::endl; 
   while (!s.empty()) 
   {
       IntervalVector box=s.top();
       s.pop();

       if (res==IBOOL::IN || res2 ==IBOOL::IN || res3 == IBOOL::IN )
       {
          in.push_back(box);
          clSIVIA::drawBox(box,echellePixel,i0,j0);
          if (is_recording||is_screening)
             clSIVIA::drawReal(box,echellePixel,i0,j0,cv::Scalar(0,204,0));
       }
       else if (res == IBOOL::OUT && res2 == IBOOL::OUT && res3 == IBOOL::OUT)
       {
          out.push_back(box);
          if (is_recording||is_screening)
             clSIVIA::drawReal(box,echellePixel,i0,j0,cv::Scalar(255,128,0));
       }
       else if (res2==IBOOL::MAYBE)
       {
          maybe.push_back(box);
          if (is_recording||is_screening)
             clSIVIA::drawReal(box,echellePixel,i0,j0,cv::Scalar(110,28,228));
       }
       else if (box.max_diam()>eps)
       {
       // get the index of the dimension of maximal size (false stands for "max")
           i = box.extr_diam_index(false);
           std::pair<IntervalVector,IntervalVector> p=box.bisect(i);
           s.push(p.first);
           s.push(p.second);
       }
       else
       {
           border.push_back(box);
       }
   }
   
   if (is_recording||is_screening){     
     if (is_recording)
        record << imageColor;
     if (is_screening){
        pthread_mutex_lock(&mutex);
        imageColor.copyTo(imageColor1);
        change = true;
        pthread_mutex_unlock(&mutex);
     }
     imageColor.setTo(cv::Scalar(0,0,0));
   }   
}

void clSIVIA::imSIVIAligth(const IntervalVector& X,const py::list& lst,double rang2,
                 const int i0,const int j0,double eps=0.1,bool efficient=true){


   int i
   std::stack<IntervalVector> s;
   s.push(X);
   std::cout << "Initialisation Finished" << std::endl; 
   while (!s.empty()) 
   {
       IntervalVector box=s.top();
       s.pop();

       if (res==IBOOL::IN || res2 ==IBOOL::IN || res3 == IBOOL::IN )
       {
          if (is_recording||is_screening)
             clSIVIA::drawReal(box,i0,j0,cv::Scalar(0,204,0));
       }
       else if (res == IBOOL::OUT && res2 == IBOOL::OUT && res3 == IBOOL::OUT)
       {

          if (is_recording||is_screening)
             clSIVIA::drawReal(box,i0,j0,cv::Scalar(255,128,0));
       }
       else if (box.max_diam()>eps)
       {
       // get the index of the dimension of maximal size (false stands for "max")
           i = box.extr_diam_index(false);
           std::pair<IntervalVector,IntervalVector> p=box.bisect(i);
           s.push(p.first);
           s.push(p.second);
       }
   }

   if (is_recording||is_screening){
     if (is_recording)
        record << imageColor;
     if (is_screening){
        pthread_mutex_lock(&mutex);
        imageColor.copyTo(imageColor1);
        change = true;
        pthread_mutex_unlock(&mutex);
     }
     imageColor.setTo(cv::Scalar(0,0,0));
   }
}

