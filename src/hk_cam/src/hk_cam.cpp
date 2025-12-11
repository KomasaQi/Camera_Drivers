/*********************************************************************
 *********************************************************************/
#include <hk_cam/hk_cam.h>

namespace hk_cam {
void HKCam::ShutDown() {
  int nRet = MV_OK;
  // 停止取流
  // end grab image
  nRet = MV_CC_StopGrabbing(handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_StopGrabbing fail! nRet [%x]", nRet);
    return;
  }

  // 关闭设备
  // close device
  nRet = MV_CC_CloseDevice(handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CloseDevice fail! nRet [%x]", nRet);
    return;
  }

  // 销毁句柄
  // destroy handle
  nRet = MV_CC_DestroyHandle(handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_DestroyHandle fail! nRet [%x]", nRet);
    return;
  }

  if(pData_) {
    free(pData_);
  }
}

bool HKCam::PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo) {
    if (NULL == pstMVDevInfo) {
        ROS_INFO("The Pointer of pstMVDevInfo is NULL!");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        ROS_INFO("CurrentIp: %d.%d.%d.%d" , nIp1, nIp2, nIp3, nIp4);
        ROS_INFO("UserDefinedName: %s" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
        ROS_INFO("Device Model Name: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        ROS_INFO("UserDefinedName: %s", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    } else {
        ROS_INFO("Not support.");
    }

    return true;
}

bool HKCam::Start(int image_width, int image_height, int framerate, int triggermode, 
  int gainval, int exposureval) {
  int nRet = MV_OK;
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  // 枚举设备
  // enum device
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
    return false;
  }

  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      ROS_INFO("[device %d]:", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (NULL == pDeviceInfo){
        break;
      } 
      PrintDeviceInfo(pDeviceInfo);            
    }
  } else {
    ROS_ERROR("Find No Devices!");
    return false;
  }

  unsigned int nIndex = 0;
  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
    return false;
  }

  // 打开设备
  // open device
  nRet = MV_CC_OpenDevice(handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]", nRet);
    return false;
  }

  // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
  if (stDeviceList.pDeviceInfo[nIndex]->nTLayerType == MV_GIGE_DEVICE) {
    int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
    if (nPacketSize > 0) {
      nRet = MV_CC_SetIntValue(handle_,"GevSCPSPacketSize",nPacketSize);
      if(nRet != MV_OK) {
        ROS_WARN("Warning: Set Packet Size fail nRet [0x%x]!", nRet);
      }
    } else {
      ROS_WARN("Warning: Get Packet Size fail nRet [0x%x]!", nPacketSize);
    }
  }

  // 设置触发模式为off
  // set trigger mode as off
  nRet = MV_CC_SetEnumValue(handle_, "TriggerMode", triggermode);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
    return false;
  }

  #if 0
  //设置图像大小
  nRet = MV_CC_SetIntValueEx(handle_, "Width", image_width);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_Set Width fail! nRet [%x]", nRet);
    return false;
  }
  nRet = MV_CC_SetIntValueEx(handle_, "Height", image_height);
  if (MV_OK != nRet){
    ROS_ERROR("MV_CC_Set Width fail! nRet [%x]", nRet);
    return false;
  }
  #endif

  //自动曝光设置
  nRet = MV_CC_SetEnumValue(handle_, "ExposureAuto", exposureval);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_Set ExposureAuto fail! nRet [%x]", nRet);
    return false;
  }

  //获取曝光时间
  MVCC_FLOATVALUE strufloatvalue = {0};
  nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &strufloatvalue);
  if (MV_OK != nRet) {
   ROS_ERROR("MV_CC_Get ExposureTime fail! nRet [%x]", nRet);
   return false;
  }
  ROS_INFO("HK ExposureTime : %lf", strufloatvalue);

  //设置曝光时间
  # if 0
  float exposuretime = 1000;
  nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", exposuretime);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_Set ExposureTime fail! nRet [%x]", nRet);
    return false;
  }
  #endif

  if (gainval > 0) {
    //自动增益
    nRet = MV_CC_SetEnumValue(handle_, "GainAuto", gainval);
    if (MV_OK != nRet){
      ROS_ERROR("MV_CC_Set GainAuto fail! nRet [%x]", nRet);
      return false;
    }
  }
  
  // 开始取流
  // start grab image
  nRet = MV_CC_StartGrabbing(handle_);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]", nRet);
    return false;
  }

  // ch:获取数据包大小 | en:Get payload size
  memset(&stParam_, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(handle_, "PayloadSize", &stParam_);
  if (MV_OK != nRet) {
      ROS_ERROR("Get PayloadSize fail! nRet [0x%x]\n", nRet);
      return false;
  }

  memset(&stImageInfo_, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  pData_ = (unsigned char *)malloc(sizeof(unsigned char) * stParam_.nCurValue);
  if (nullptr == pData_){
    ROS_ERROR("malloc fail!");
    return false;
  }

  return true;
}

bool HKCam::GrabImage(sensor_msgs::Image* msg) {
  int nRet = MV_CC_GetOneFrameTimeout(handle_, pData_, stParam_.nCurValue, &stImageInfo_, 1000);
  msg->header.stamp = ros::Time::now();
  if (nRet == MV_OK) {
      unsigned char *pDataForRGB = nullptr;
      pDataForRGB = (unsigned char*)malloc(stImageInfo_.nWidth * stImageInfo_.nHeight * 4 + 2048);
      if (nullptr == pDataForRGB) {
        ROS_ERROR("malloc fail!");
        return false;
      }

      // 像素格式转换
      // convert pixel format 
      MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
      // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
      // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
      // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format, 
      // destination pixel format, output data buffer, provided output buffer size
      stConvertParam.nWidth = stImageInfo_.nWidth;
      stConvertParam.nHeight = stImageInfo_.nHeight;
      stConvertParam.pSrcData = pData_;
      stConvertParam.nSrcDataLen = stImageInfo_.nFrameLen;
      stConvertParam.enSrcPixelType = stImageInfo_.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
      stConvertParam.pDstBuffer = pDataForRGB;
      stConvertParam.nDstBufferSize = stImageInfo_.nWidth * stImageInfo_.nHeight *  4 + 2048;
      nRet = MV_CC_ConvertPixelType(handle_, &stConvertParam);
      if (MV_OK != nRet) {
        ROS_ERROR("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
        return false;
      }

      void *imgdata = nullptr;
      imgdata = (unsigned char*)malloc(stConvertParam.nDstLen);
      if (nullptr == imgdata) {
        ROS_ERROR("malloc imgdata fail!");
        return false;
      }
      memcpy(imgdata, pDataForRGB ,stConvertParam.nDstLen);
      fillImage(*msg, "bgr8", stImageInfo_.nHeight, stImageInfo_.nWidth, 3 * stImageInfo_.nWidth, imgdata);

      if (pDataForRGB) {
        free(pDataForRGB);
      }
      if (imgdata) {
        free(imgdata);
      }
  } else{
    ROS_ERROR("No data[%x]\n", nRet);
    return false;
  }
  
  return true;
}

}
