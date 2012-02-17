#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <OpenGl/gl.h>

#include <string>
#include <sstream>

#include "disparity_node.h"
#include "calibrate_node.h"


/*
void on_opengl(void* param)
{
	glLoadIdentity();
	
	glTranslated(0.0, 0.0, -1.0);
	
	glRotatef( 55, 1, 0, 0 );
	glRotatef( 45, 0, 1, 0 );
	glRotatef( 0, 0, 0, 1 );
	
	static const int coords[6][4][3] = {
		{ { +1, -1, -1 }, { -1, -1, -1 }, { -1, +1, -1 }, { +1, +1, -1 } },
		{ { +1, +1, -1 }, { -1, +1, -1 }, { -1, +1, +1 }, { +1, +1, +1 } },
		{ { +1, -1, +1 }, { +1, -1, -1 }, { +1, +1, -1 }, { +1, +1, +1 } },
		{ { -1, -1, -1 }, { -1, -1, +1 }, { -1, +1, +1 }, { -1, +1, -1 } },
		{ { +1, -1, +1 }, { -1, -1, +1 }, { -1, -1, -1 }, { +1, -1, -1 } },
		{ { -1, -1, +1 }, { +1, -1, +1 }, { +1, +1, +1 }, { -1, +1, +1 } }
	};
	
	for (int i = 0; i < 6; ++i) {
		glColor3ub( i*20, 100+i*10, i*42 );
		glBegin(GL_QUADS);
		for (int j = 0; j < 4; ++j) {
			glVertex3d(0.2 * coords[i][j][0], 0.2 * coords[i][j][1], 0.2 * coords[i][j][2]);
		}
		glEnd();
	}
}
*/

class Filter {
	
public:
	
	Filter(ros::NodeHandle &n) :
	node(n), transport(n), disparityNode(320,240,DisparityNode::STEREO_SGBM)
	{
		
		//cv::namedWindow("depth",1);
		//cv::namedWindow("rgb",1);
		
		debug = true;
		
#if 1
    std::string left_str = "/fake_stereo/left/image_raw";
    std::string right_str = "/fake_stereo/right/image_raw";
#else
    std::string left_str = "/narrow_stereo_textured/left/image_raw";
    std::string right_str = "/narrow_stereo_textured/right/image_raw";
#endif
    
		//depth_sub = transport.subscribe("kinect_camera/depth", 1, &Filter::depthCB, this, image_transport::TransportHints("compressed"));
		left_sub = transport.subscribe(left_str.c_str(), 1, &Filter::leftCB, this); //, image_transport::TransportHints("compressed"));
		right_sub = transport.subscribe(right_str.c_str(), 1, &Filter::rightCB, this); //, image_transport::TransportHints("compressed"));
		
		
	}
	
	~Filter()
	{
		;
	}
	
	
	inline bool getDebug(void) const { return debug; }
	
	bool genericCB(const sensor_msgs::ImageConstPtr& msg, 
						cv::Mat& image,
						const char* windowName){
    
    //ROS_INFO("%s",windowName);
    //return true;
		
		cv_bridge::CvImagePtr cv_msg;
		
		try
		{
			cv_msg = cv_bridge::toCvCopy(msg, "bgr8");
			//cv_msg = cv_bridge::toCvCopy(msg, "gray");
			//image = cv_msg->image;
      cv_msg->image.copyTo(image);
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("Error getting %s",windowName);
			return false;
		}
		/*
		// display image if in debug mode
		if(0){
			cv::imshow("rgb",image);
			cv::waitKey(10);
		}
		*/
		return true;
	}
	
	
	void rightCB(const sensor_msgs::ImageConstPtr& msg){
		genericCB(msg,right,"right");
	}
	
	//void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr,
	//						 const sensor_msgs::CameraInfoConstPtr& info_msg)
	void leftCB(const sensor_msgs::ImageConstPtr& msg){	
		genericCB(msg,left,"left");
	}
  
  void spin(){
    cv::Mat depth,l,r;
    bool ok = true;
    ros::Rate rate(10);
    
		cv::namedWindow("depth",1);
    cv::namedWindow("left",1);
    cv::namedWindow("right",1);
    
    ok = calNode.getParameters("intrinsics.yml","extrinsics.yml");
    //disparityNode.setQ( calNode.getQ() );
    
    if(!ok){
      node.shutdown();
      exit(1);
    }
    
    while( node.ok() ){
      ok = calNode.rectifyImage(left,right,l,r);
      
      //l = left;
      //r = right;
      //ok = true;
      
      if(ok) ok = disparityNode.disparity(l,r,depth);
      else ROS_ERROR("disparityNode.disparity()");
      
      if(ok){
        //cv::imshow("left",l);
        //cv::imshow("right",r);
        cv::imshow("depth",depth);
        cv::waitKey(10);
      }
      //else ROS_ERROR("disparity error");
      
      ros::spinOnce();
      rate.sleep();
    }
  }
	
protected:
	
	ros::NodeHandle node;
	image_transport::ImageTransport transport;
	image_transport::Subscriber right_sub;
	image_transport::Subscriber left_sub;
  
  DisparityNode disparityNode;
  CalibrateNode calNode;
  
  cv::Mat left, right;
  
	bool debug;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter");
	ros::NodeHandle n;
	Filter filter(n);
	filter.spin();
	return 0;
}
