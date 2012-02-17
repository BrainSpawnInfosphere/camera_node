
#ifndef __CALIBRATE_NODE_H__
#define __CALIBRATE_NODE_H__

#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>


class CalibrateNode {
public:
  
  enum {CV_SQUARES, CV_DOTS};
  
  CalibrateNode(int type=CV_SQUARES){
    ;
  }
  
  bool findGrid(){
    return true;
  }
  
  bool calcParameters(){
    return true;
  }
  
  bool getParameters(std::string intrinsic_filename, std::string extrinsic_filename){
    if( 1 )
    {
      // reading intrinsic parameters
      cv::FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
      if(!fs.isOpened())
      {
        ROS_ERROR("Failed to open file %s\n", intrinsic_filename.c_str());
        return false;
      }
      
      cv::Mat M1, D1, M2, D2;
      fs["M1"] >> M1;
      fs["D1"] >> D1;
      fs["M2"] >> M2;
      fs["D2"] >> D2;
      
      fs.open(extrinsic_filename, CV_STORAGE_READ);
      if(!fs.isOpened())
      {
        ROS_ERROR("Failed to open file %s\n", extrinsic_filename.c_str());
        return false;
      }
      
      cv::Mat R, T, R1, P1, R2, P2;
      fs["R"] >> R;
      fs["T"] >> T;
      
      cv::Size img_size;
      img_size.width = 320;
      img_size.height = 240;
      
      cv::Rect roi1, roi2;
      
      stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );
      
      //cv::Mat map11, map12, map21, map22;
      initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
      initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
      /*
      Mat img1r, img2r;
      remap(img1, img1r, map11, map12, INTER_LINEAR);
      remap(img2, img2r, map21, map22, INTER_LINEAR);
      
      img1 = img1r;
      img2 = img2r;
      */
      
      ROS_INFO("Loaded calibration files");
      
    }
    return true;
  }
  
  bool rectifyImage(const cv::Mat& l, const cv::Mat& r, cv::Mat& cl, cv::Mat& cr){
    //Mat img1r, img2r;
    if( l.data == 0 || r.data == 0) return false;
    
    remap(l, cl, map11, map12, cv::INTER_LINEAR);
    remap(r, cr, map21, map22, cv::INTER_LINEAR);
    
    return true;
  }


  inline cv::Mat& getQ(){ return Q; }
  
protected:
  
  //cv::Mat M1, D1, M2, D2; // intrinsic parameters
  //cv::Mat R, T, R1, P1, R2, P2; // extrinsic parameters
  cv::Mat map11, map12, map21, map22;
  cv::Mat Q;
};



#endif