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
 * Author: Kevin J. Walchko on 10/17/2011
 *********************************************************************
 *
 * Simple stereo camera driver using OpenCV interface to grab images.
 *
 * rosrun camera_node opencv_stereo _source:=# _size:=#x# _debug:=true/false
 *		where:
 *			source: > 0 selects a camera other than default
 *          size: my MacBook Pro can do 160x120, 320x240, 640x480
 *          fps: my MacBook Pro seems to ignore fps and always gives me ~15 fps
 *
 * Example:
 * rosrun camera_node opencv_stereo _debug:=true _size:=160x120 _fps:=30
 *
 * Change Log:
 * 17 Oct 2011 Created, based off uvc_camera stereo driver
 *
 **********************************************************************
 *
 * 
 *
 */


#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "CameraNode.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

namespace uvc_camera {
  
  class StereoCamera {
  public:
    StereoCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(ros::Time time);
    void feedImages();
    ~StereoCamera();
    
  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;
    
    CameraNode *cam_left, *cam_right;
    int width, height, fps, skip_frames, frames_to_skip;
    int left_device, right_device;
    std::string frame;
    bool rotate_left, rotate_right;
    
    CameraInfoManager left_info_mgr, right_info_mgr;
    
    image_transport::Publisher left_pub, right_pub;
    ros::Publisher left_info_pub, right_info_pub;
    
    boost::thread image_thread;
  };
  
};

