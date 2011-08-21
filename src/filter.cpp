#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> // handles raw or compressed images
#include <cv_bridge/CvBridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>

class Filter {
	
public:
	
	Filter(ros::NodeHandle &n) :
	node(n), transport(n)
	{
    debug = true;
    
		image_pub = transport.advertise("image",1);
		
    cv::namedWindow("Debug",1);
		image_sub = transport.subscribe("opencv_cam/camera", 1, &Filter::imageCallback, this, image_transport::TransportHints("compressed"));
		//image_sub = transport.subscribeCamera("axis_camera", 1, &Filter::imageCallback, this);
		
		
	}
	
	~Filter()
	{
		cvDestroyWindow("Image window");
	}
  
  inline bool getDebug(void) const { return debug; }
	
	//void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr,
	//						 const sensor_msgs::CameraInfoConstPtr& info_msg)
	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		
    cv_bridge::CvImagePtr cv_msg;
    
		try
		{
			cv_msg = cv_bridge::toCvCopy(msg, "bgr8");
		}
		catch (sensor_msgs::CvBridgeException& error)
		{
			ROS_ERROR("error");
			return;
		}
		
    // display image if in debug mode
    if(getDebug()){
      imshow("Debug",cv_msg->image);
      int key = cv::waitKey(100);
      if(key == 's'){
        static int picNum = 0;
        std::stringstream ss;
        ss << picNum++;
        imwrite("image_" + ss.str() + ".jpg",cv_msg->image);
        ROS_INFO("Saved image");
      }
    }
		
		try
		{
      
      //ros::Time time = ros::Time::now();
      
      // convert OpenCV image to ROS message
			image_pub.publish(cv_msg->toImageMsg());
		}
		catch (sensor_msgs::CvBridgeException error)
		{
			ROS_ERROR("error");
		}
		
	}
		
protected:
	
	ros::NodeHandle node;
	image_transport::ImageTransport transport;
	image_transport::Subscriber image_sub;
	//sensor_msgs::CvBridge bridge;
	image_transport::Publisher image_pub;
	
  bool debug;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter");
	ros::NodeHandle n;
	Filter filter(n);
	ros::spin();
	return 0;
}
