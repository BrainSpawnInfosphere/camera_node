#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h" // handles raw or compressed images
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "GraphUtils.h"


class ImageConverter {
	
public:
	
	ImageConverter(ros::NodeHandle &n) :
	n_(n), it_(n_)
	{
		image_pub_ = it_.advertise("image",1);
		
		cvNamedWindow("Image window");
		image_sub_ = it_.subscribe("axis_camera", 1, &ImageConverter::imageCallback, this, image_transport::TransportHints("compressed"));
		//image_sub_ = it_.subscribeCamera("axis_camera", 1, &ImageConverter::imageCallback, this);
		
		
	}
	
	~ImageConverter()
	{
		cvDestroyWindow("Image window");
	}
	
	//void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr,
	//						 const sensor_msgs::CameraInfoConstPtr& info_msg)
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
	{
		
		IplImage *cv_image = NULL;
		try
		{
			cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("error");
			return;
		}
		
#if 0
		// display window showing results
		cvShowImage("Image window", cv_image);
		//cvWaitKey(3);		
#endif
		
		try
		{
			image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}
		
	}
		
protected:
	
	ros::NodeHandle n_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	sensor_msgs::CvBridge bridge_;
	image_transport::Publisher image_pub_;
	
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle n;
	ImageConverter ic(n);
	ros::spin();
	return 0;
}
