
#ifndef __DISPARITY_NODE_H__
#define __DISPARITY_NODE_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>


class DisparityNode {
public:
  
  //--- Calculate Disparity ----
  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3 };
  
  DisparityNode(int width=320, int height=240, int type=STEREO_SGBM, bool gray=true){
    alg = type;
    convertToGray = (gray || alg == STEREO_BM);
    
    cv::Rect roi1, roi2;
    roi1.width = width;
    roi1.height = height;
    roi2 = roi1;
    
    // remmove this global!!! FIXME 241011 kjw
    numberOfDisparities = 16*2; //((width/8) + 15) & -16;
    
		int SADWindowSize = 1;
    int speckleWindowSize = 100;
    int speckleRange = 32;
    
		if(numberOfDisparities % 16 != 0) ROS_ERROR("The max disparity must be a positive integer divisible by 16");
		if(SADWindowSize % 2 != 1) ROS_ERROR("The block size must be a positive odd number");
    
    switch(alg){
      case STEREO_BM:
        bm.state->roi1 = roi1;
        bm.state->roi2 = roi2;
        bm.state->preFilterCap = 31;
        bm.state->SADWindowSize = SADWindowSize > 4 ? SADWindowSize : 9;
        bm.state->minDisparity = 0;
        bm.state->numberOfDisparities = numberOfDisparities;
        bm.state->textureThreshold = 10;
        bm.state->uniquenessRatio = 15;
        bm.state->speckleWindowSize = speckleWindowSize;
        bm.state->speckleRange = 32;
        bm.state->disp12MaxDiff = 1;
        break;
      case STEREO_HH:
      case STEREO_SGBM:
        int cn;
        // how many channels?
        if(convertToGray) cn = 1;
        else cn = 3;
        
        sgbm.preFilterCap = 63;
        sgbm.SADWindowSize = SADWindowSize > 2 ? SADWindowSize : 3;
        
        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity = 0;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 10;
        sgbm.speckleWindowSize = speckleWindowSize;
        sgbm.speckleRange = speckleRange;
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = (alg == STEREO_HH);
        
        break;
      case STEREO_VAR:
        var.levels = 3;									  // ignored with USE_AUTO_PARAMS
        var.pyrScale = 0.5;								// ignored with USE_AUTO_PARAMS
        var.nIt = 25;
        var.minDisp = -numberOfDisparities;	
        var.maxDisp = 0;
        var.poly_n = 3;
        var.poly_sigma = 0.0;
        var.fi = 15.0f;
        var.lambda = 0.03f;
        var.penalization = var.PENALIZATION_TICHONOV;	// ignored with USE_AUTO_PARAMS
        var.cycle = var.CYCLE_V;						// ignored with USE_AUTO_PARAMS
        var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
      
        break;
    }
  }
  
  /**
   * Calculate the disparity given two images.
   * \input cv::Mat left/right can be gray or color image
   * \output cv::Mat disparity will be a gray image of CV_16S (16b signed)
   *
   * \todo don't blindly convert images to gray, determine if they are gray already
   */
  bool disparity(cv::Mat& l, cv::Mat& r, cv::Mat& disp8){
    
    if(!(l.rows == r.rows && l.cols == r.cols)) return false;
    if( l.data == 0 || r.data == 0) return false;
    
    cv::Mat left;
    cv::Mat right;
    /*
    // are they gray?? FIXME 241011 kjw
    if(convertToGray){ // gray
      if( !(l.type() == CV_8UC1) ) cv::cvtColor(l, left, CV_BGR2GRAY); //l.convertTo(left, CV_8UC1);
      if( !(r.type() == CV_8UC1) ) cv::cvtColor(r, right, CV_BGR2GRAY); //r.convertTo(right, CV_8UC1);
      ROS_INFO("go gray");
    }
    else { // color
      left = l;
      right = r;
    }
    */
    
    cv::cvtColor(l, left, CV_BGR2GRAY); left.convertTo(left, CV_8UC1);
    cv::cvtColor(r, right, CV_BGR2GRAY); right.convertTo(right, CV_8UC1);
    
    //ROS_INFO("gray: %d %d",convertToGray ? 1 : 0, left.type() == CV_8UC1 ? 1 : 0);
    //ROS_INFO("left/right channels: %d %d",left.channels(),right.channels());
    		
    cv::Mat disp;
    disp.create(left.rows, left.cols, CV_8UC1);
    
		if( alg == STEREO_BM )
			bm(left, right, disp);
		else if( alg == STEREO_VAR ) {
			var(left, right, disp);
		}
		else if( alg == STEREO_SGBM || alg == STEREO_HH )
			sgbm(left, right, disp);
    else
      ROS_ERROR("unknown algorithm (alg) in DisparityNode::disparity()");
    
    colorizeDepth(disp,disp8); // remove from this function!!!
		
    return true;
  }
  
	/**
	 * Colorizes the the CV_16SC1 cv::Mat raw depth image to a more human
	 * readable CV_8UC3 cv:Mat image for display.
	 */
	bool colorizeDepth(const cv::Mat& d, cv::Mat& image)
	{
		
    
		// check depth is CV_16UC1 and image is CV_8UC3, both 640x480
		if(0){ 
			return false;
		}
    
    // make sure image is 8b color image
    //image.create(d.rows, d.cols, CV_32FC3);
    //image.create(d.rows, d.cols, CV_8UC3);
    
    //reprojectImageTo3D(d,image,Q);
    
    //image.convertTo(image,CV_8UC3);
		
    //d.copyTo(image);
    //d.convertTo(image,CV_8U, 255./(numberOfDisparities*16.));
    d.convertTo(image,CV_8U, 255./(numberOfDisparities*16));
    //d.convertTo(image,CV_8U);
    return true;
    
    
    cv::Mat depth;
    d.convertTo(depth,CV_16UC1,65535/(8*16));
		
		// colorize
		unsigned char *depth_mid = (unsigned char*)(image.data);
		const int len = depth.rows*depth.cols;
    int i;
		for (i = 0; i < len; i++) {
			int lb = ((short *)depth.data)[i] % 256;
			int ub = ((short *)depth.data)[i] / 256;
			switch (ub) {
				case 0:
					depth_mid[3*i+2] = 255;
					depth_mid[3*i+1] = 255-lb;
					depth_mid[3*i+0] = 255-lb;
					break;
				case 1:
					depth_mid[3*i+2] = 255;
					depth_mid[3*i+1] = lb;
					depth_mid[3*i+0] = 0;
					break;
				case 2:
					depth_mid[3*i+2] = 255-lb;
					depth_mid[3*i+1] = 255;
					depth_mid[3*i+0] = 0;
					break;
				case 3:
					depth_mid[3*i+2] = 0;
					depth_mid[3*i+1] = 255;
					depth_mid[3*i+0] = lb;
					break;
				case 4:
					depth_mid[3*i+2] = 0;
					depth_mid[3*i+1] = 255-lb;
					depth_mid[3*i+0] = 255;
					break;
				case 5:
					depth_mid[3*i+2] = 0;
					depth_mid[3*i+1] = 0;
					depth_mid[3*i+0] = 255-lb;
					break;
				default:
					depth_mid[3*i+2] = 0;
					depth_mid[3*i+1] = 0;
					depth_mid[3*i+0] = 0;
					break;
			}
		}
		return true;
	}
  
  bool save(std::string file){
    /*
      imwrite(disparity_filename, disp8);
    
      ROS_INFO("storing the point cloud...");
      fflush(stdout);
    cv::Mat xyz;
      reprojectImageTo3D(disp, xyz, Q, true);
      saveXYZ(point_cloud_filename, xyz);
      printf("\n");
     */
    return true;
  }
  /*
  // remove if not doing reprojectImageTo3D()
  void setQ(cv::Mat& q){
    Q = q;
  }
  */                     
                       
  
protected:
	// stereo algorithms
	cv::StereoBM bm;
	cv::StereoSGBM sgbm;
	cv::StereoVar var;
  
  //cv::Mat Q;
  
  bool convertToGray;
  int numberOfDisparities; // remove me
  //cv::Rect roi1, roi2;
  
  int alg; // algorithm being used
};



#endif