#ifndef __SIVIA_HPP__
#define __SIVIA_HPP__

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/videoio/videoio.hpp"
#include <pthread.h>
#include <stdexcept>

enum IBOOL { IN = 0 , OUT = 1 ,MAYBE =2 , UNK = 3 , EMPTY = 4 , UNK2 = 5 };

class clSIVIA
{
  public:
      clSIVIA(int width,int heigth,echellePix) {
       is_recording = false;
       is_rect = false;
       show = false;
       echellePixel = echellePix;
       width_ = width;
       height_ = height;
       imageColor = cv::Mat(height_,width_,CV_8UC3);
       imageColor1 = cv::Mat(height_,width_,CV_8UC3);
       imageColor2 = cv::Mat(height_,width_,CV_8UC3);
       imageColor.setTo(cv::Scalar(0,0,0));
       imageColor1.setTo(cv::Scalar(0,0,0));
       imageColor2.setTo(cv::Scalar(0,0,0));
       std::cout << "clSIVIA initialised !" << std::endl;
      }
      py::list imSIVIA(const IntervalVector& X,const py::list& lst,double rang2,const float echellePixel,
                 const int i0,const int j0,double eps,bool efficient);
      void imSIVIAligth(const IntervalVector& X,const py::list& lst,double rang2,const float echellePixel,
                 const int i0,const int j0,double eps,bool efficient);
      void setRecord(std::string const & msg,float fps,bool rectangle);
      void startScreening(bool rectangle);
      void stopScreening();
      ~clSIVIA(){
        show = false;
        if (is_recording)
           pthread_join(thread,NULL);
      }
  private:
     pthread_mutex_t mutex;
     pthread_t thread;
     bool change;
     cv::Mat imageColor;
     cv::Mat imageColor1;
     cv::Mat imageColor2;
     cv::VideoWriter record;
     float echellePixel;
     bool is_recording;
     bool is_screening;
     bool is_rect;
     int width_;
     int height_;
     bool show;
     void drawBox(const IntervalVector& X,const float echellePixel,const int i0,const int j0);
     void drawReal(const IntervalVector& X,const float echellePixel,const int i0,const int j0,cv::Scalar color);
     void screening();
     static void *screening_helper(void *context)
     {
        ((clSIVIA *)context)->screening();
        return NULL;
     }
};


#endif // __SIVIA_HPP__
