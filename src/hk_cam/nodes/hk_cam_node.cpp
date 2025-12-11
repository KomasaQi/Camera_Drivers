/*********************************************************************
*
*********************************************************************/

#include "ros/ros.h"
#include <hk_cam/hk_cam.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <iostream>

namespace hk_cam {

class HKCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_;
  image_transport::Publisher image_pub_;

  int image_width_, image_height_, framerate_, exposure_, triggermode_, gainval_;
  std::string camera_name_;
  bool camstate_ = false;

  HKCam cam_;

  HKCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertise("/hk/image_rect", 1);

    node_.param("image_width", image_width_, 1920);
    node_.param("image_height", image_height_, 1080);
    node_.param("framerate", framerate_, 10);
    node_.param("exposureval", exposure_, 2);
    node_.param("triggermode", triggermode_, 0);
    node_.param("gainval", gainval_, 2);
    
    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("hk"));
    node_.param("camera_name", camera_name_, std::string("hk"));

    ROS_INFO("Starting '%s' at %dx%d %i FPS", camera_name_.c_str(), image_width_, image_height_, framerate_);

    // start the camera
    camstate_ = cam_.Start(image_width_, image_height_, framerate_, triggermode_, gainval_, exposure_);
  }

  virtual ~HKCamNode() {
    cam_.ShutDown();
  }

  bool SendImage() {
    // grab the image
    cam_.GrabImage(&img_);

    // publish the image
    image_pub_.publish(img_);

    return true;
  }

  bool Spin() {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok()) {
      if (!SendImage()) {
        ROS_WARN("HK camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();
    }
    return true;
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hk_cam");
  hk_cam::HKCamNode hk;
  if (hk.camstate_) {
    hk.Spin();
  }

  return true;
}
