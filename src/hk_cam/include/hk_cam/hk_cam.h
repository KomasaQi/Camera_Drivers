/*********************************************************************
 *********************************************************************/
#ifndef HK_CAM_H
#define HK_CAM_H

#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "MvCameraControl.h"
#include "ros/ros.h"

namespace hk_cam {

class HKCam {
 public:
  HKCam(){};
  ~HKCam(){};
  void ShutDown();
  bool GrabImage(sensor_msgs::Image* msg);
  bool Start(int image_width, int image_height, int framerate, int triggermode, int gainval, int exposureval);

 private:
  bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo);

  void* handle_ = NULL;
  MVCC_INTVALUE stParam_;
  MV_FRAME_OUT_INFO_EX stImageInfo_ = {0};
  unsigned char *pData_ = nullptr;
};

}

#endif

