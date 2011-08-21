/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 6/20/2011
 *********************************************************************
 *
 * Simple camera driver using OpenCV 2.2 interface to grab images.
 *
 * rosrun camera_node opencv_cam _source:=# _size:=#x# _debug:=true/false
 *		where:
 *			source: > 0 selects a camera other than default
 *          size: my MacBook Pro can do 160x120, 320x240, 640x480
 *          fps: my MacBook Pro seems to ignore fps and always gives me ~15 fps
 *
 * Example:
 * rosrun camera_node opencv_cam _debug:=true _size:=160x120 _fps:=30
 *
 * Change Log:
 * 20 June 2011 Created
 *  3 Aug 2011 Major rewrite to use more of the updated methods for E Turtle
 *
 **********************************************************************
 *
 * 
 *
 */


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h> // depreciated
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/cv_bridge.h> // switches between cv and ros image formats 
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;


/**
 * Simple class that grabs the default camera and sends it over ROS's
 * image_transport. Also sends camera info for calibration. OpenCV doesn't
 * have the greatest capabilities to access cameras, but it is a good
 * cross platform solution with acceptable performance given the generic
 * capabilities.
 */
class CameraNode {
	
public:
	
	CameraNode(ros::NodeHandle &n) :
  transport(node), info_mgr(node), node(n), fpsUpdate(30*5)
	{
		// the only way i have figured out how to clean up is to delete the 
		// params, otherwise they are always there or need to kill roscore
		node.param<bool>("debug",debug, false);		node.deleteParam("debug");
		node.param("source",source, 0);	node.deleteParam("source");
		//node.param("width",width, 320);	node.deleteParam("width");
		//node.param("height",height, 240); node.deleteParam("height");
    node.param<double>("fps",fps, 30.0);	node.deleteParam("fps"); // doesn't work
    hertzLoop = fps;
    
    std::string res;
    node.param<std::string>("size",res, "320x240");	node.deleteParam("size");
    
    if( !res.compare("640x480") ){
      width = 640;
      height = 480;
    }
    else if( !res.compare("160x120") ){
      width = 160;
      height = 120;
    }
    else { // default
      width = 320;
      height = 240;
    }
		
		camera.open(source);
    
    info_mgr.setCameraName("default"); // this takes a std::string
    
		if(!camera.isOpened()){
			ROS_ERROR("CameraNode: couldn't open camerea");
			node.shutdown();
			return;
		}
		
    
		camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
		camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		camera.set(CV_CAP_PROP_FPS, fps);
		
    
    camera >> cv_image;
    
    fps = camera.get(CV_CAP_PROP_FPS); // this doesn't seem to work
		width = camera.get(CV_CAP_PROP_FRAME_WIDTH);
		height = camera.get(CV_CAP_PROP_FRAME_HEIGHT);
		
		image_pub = transport.advertiseCamera("camera",1);
		//image_pub = transport.advertise("camera",1);
    
    ROS_INFO("CameraNode: opened camera[%d] %dx%d @ %d fps %s",
             source,width,height,(int)fps,(debug ? "in debug mode" : ""));
    
    if(getDebug()) namedWindow("Debug",1);
    
    frameCount = 0;
    frameTime = ros::Time::now();
		
	}
	
	~CameraNode()
	{
		
	}
	
	inline double getFPS(void) const { return fps; } // always returns 0
	
	inline bool getDebug(void) const { return debug; }
	
	bool imageCallback(void)
	{
    //static int imageNumber = 1;
		// grab image ------------------------------------------------------
		try
		{
			camera >> cv_image;
			
			// display image if in debug mode
			if(getDebug()){
        imshow("Debug",cv_image);
        int key = cv::waitKey(100);
        if(key == 's'){
          imwrite("image.jpg",cv_image);
          ROS_INFO("Saved image");
        }
      }
		}
		catch (...)
		{
			ROS_ERROR("CameraNode::imageCallback(): Couldn't grab image");
			return false;
		}
		
		// send image and camera params ------------------------------------
		try
		{
      
			// if subscribers, pub images
			// there is no easy way to send cv::Mat, so convert to IplImage
			if(image_pub.getNumSubscribers() > 0){
        
        ros::Time time = ros::Time::now();
        
        // convert OpenCV image to ROS message
        //cv_bridge::CvImage cvi;			
        cvi.header.stamp = time;
        cvi.header.frame_id = "camera";
        cvi.encoding = "bgr8";
        //cvi.image = cv_image.clone();
        cvi.image = cv_image;
        //cv_image.copyTo(cvi.image);
        //ROS_INFO("1");
        
        // get camera parameters
				CameraInfo info = info_mgr.getCameraInfo();
				info.header.stamp = cvi.header.stamp;
				info.header.frame_id = cvi.header.frame_id; // whatever it is called
        info.width = cv_image.cols;
        info.height = cv_image.rows;
        //ROS_INFO("2");
        
        // chose not to do pointers so need one more conversion
        //publish(sensor_msgs::Image&, sensor_msgs::CameraInfo&)
        sensor_msgs::Image im;
        cvi.toImageMsg(im);
				image_pub.publish(im,info);
        
        
        //sensor_msgs::ImagePtr im = cvi.toImageMsg();
        //CameraInfoPtr ip = &
        //image_pub.publish(cvi.toImageMsg());
        //ROS_INFO("3");
        
        if(frameCount == fpsUpdate && getDebug()){
          ros::Duration diff = time - frameTime;
          frameTime = time;
          frameCount = 0;
          
          fps = double(fpsUpdate)/diff.toSec();
          
          ROS_INFO("FPS: %f",fps);
        }
        else ++frameCount;
			}
		}
		catch (...)
		{
			ROS_ERROR("imageCallback(): Couldn't publish image");
			return false;
		}
		
		return true;
		
	}
	
	/**
	 * Loop forever until user hits ctrl-c to end
	 */
	void spin(void){
		
		ros::Rate r( hertzLoop ); // spin
		
		while(node.ok()){
			imageCallback();
			
			ros::spinOnce();
			r.sleep();
		}
	}
	
protected:
	
	ros::NodeHandle node;
	image_transport::ImageTransport transport;
	//sensor_msgs::CvBridge bridge;
	image_transport::CameraPublisher image_pub;
	//image_transport::Publisher image_pub;
  cv_bridge::CvImage cvi;		
	
	//ros::Publisher info_pub;
	CameraInfoManager info_mgr; // depreciated
	
	VideoCapture camera;
	
	bool debug;	// debug mode ... should be boolean
	int source; // source of images
	int width;	// image width/cols
	int height;	// image height/rows
	double fps;		// frame rate
  double hertzLoop;
	
  cv::Mat cv_image;	// image from camera
  unsigned int frameCount;
  ros::Time frameTime;
  const unsigned int fpsUpdate;
	
};

/**
 * Still having problems with pass commandline args ... the param server
 * remembers the last params you put on it. You need to kill roscore to 
 * clear the param server.
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "opencv_cam");
	ros::NodeHandle n("~");
	
  CameraNode ic(n);
  ic.spin();
  
	return 0;
}
