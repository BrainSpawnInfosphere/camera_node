#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h> 
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/cv_bridge.h> // switches between cv and ros image formats .. doesn't work
#include <cv_bridge/CvBridge.h> // old way
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "GraphUtils.h"

using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;


/**
 * Simple class that grabs the default camera and sends it over ROS's
 * image_transport.
 */
class CameraNode {
	
public:
	
	CameraNode(ros::NodeHandle &n) :
	node(n), transport(node), info_mgr(node)
	{
		// the only way i have figured out how to clean up is to delete the 
		// params, otherwise they are always there or need to kill roscore
		node.param("debug",debug, 0);		node.deleteParam("debug");
		node.param("source",source, 0);	node.deleteParam("source");
		node.param("width",width, 320);	node.deleteParam("width");
		node.param("height",height, 240); node.deleteParam("height");
		
		//camera = cvCaptureFromCAM (source);		//capture stream
		camera.open(source);
		if(!camera.isOpened()){
			ROS_ERROR("CameraNode: couldn't open camerea");
			node.shutdown();
			return;
		}
		else {
			ROS_INFO("CameraNode: opened camera[%d] %dx%d %s",
						source,width,height,(debug>0 ? "in debug mode" : ""));
		}
		
		camera.set(CV_CAP_PROP_FRAME_WIDTH, width);
		camera.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		
		fps = static_cast<int> camera.get(CV_CAP_PROP_FPS);
		
		//cvQueryFrame (camera);
		
		image_pub = transport.advertise("camera",1);
		info_pub = node.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
		
		grabError = pubError = 0;
	}
	
	~CameraNode()
	{
		//cvReleaseCapture (&camera);
	}
	
	inline bool getDebug(void) const { return (debug > 0 ? true : false); }
	
	bool imageCallback(void)
	{
		// grab image ------------------------------------------------------
		try
		{
			camera >> cv_image;
			
			// display image if in debug mode
			if(getDebug()) imshow("Debug",cv_image);
		}
		catch (...)
		{
			++grabError;
			ROS_ERROR("[%i]imageCallback(): Couldn't grab image", grabError);
			return false;
		}
		
		// send image ------------------------------------------------------
		try
		{
			// if subscribers, pub images
			// there is no easy way to send cv::Mat, so convert to IplImage
			if(image_pub.getNumSubscribers() > 0){
				IplImage ipl_image = cv_image;
				image_pub.publish(bridge.cvToImgMsg(&ipl_image, "bgr8"));
				//cv_bridge::CvImage cv_im;
				//cv_im.header = ?? where does header info come from? need time stamp
				//cv_im.encoding = CV_8UC3; // bgr8
				//cv_im.image = cv_image;
				//cv_image.copyTo(cv_im.image);
				//image_pub.publish(cv_im.toImageMsg());
			}
			
			// if subscribers, pub info
			if(info_pub.getNumSubscribers() > 0){
				CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));
				
				
				// Throw out any CamInfo that's not calibrated to this camera mode 
				if (info->K[0] != 0.0 &&
					 (cv_image.cols != info->width
					  || cv_image.rows != info->height)) {
						 info.reset(new CameraInfo());
					 }
				
				// If we don't have a calibration, set the image dimensions 
				if (info->K[0] == 0.0) {
					info->width = cv_image.cols;
					info->height = cv_image.rows;
				}
				
				
				ros::Time time = ros::Time::now();
				info->header.stamp = time;
				info->header.frame_id = "camera"; // whatever it is called
				
				info_pub.publish(info);
			}
		}
		catch (...)
		{
			++pubError;
			ROS_ERROR("[%i]imageCallback(): Couldn't publish image",pubError);
			return false;
		}
		
		return true;
		
	}
	
protected:
	
	ros::NodeHandle node;
	image_transport::ImageTransport transport;
	sensor_msgs::CvBridge bridge;
	image_transport::Publisher image_pub;
	
	ros::Publisher info_pub;
	//sensor_msgs::CameraInfo cam_info;
	CameraInfoManager info_mgr;
	
	VideoCapture camera;
	unsigned int pubError; // number of publisher errors -- delete?
	unsigned int grabError;// number of grab errors -- delete?
	
	int debug;	// debug mode ... should be boolean
	int source; // source of images
	int width;	// image width/cols
	int height;	// image height/rows
	int fps;		// frame rate
	
	Mat cv_image;	// image from camera
	
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
	
	if(ic.getDebug()) namedWindow("Debug",1); //cvNamedWindow("Debug");
	
	ros::Rate r(30);
	
	while(n.ok()){
		ic.imageCallback();
		
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
