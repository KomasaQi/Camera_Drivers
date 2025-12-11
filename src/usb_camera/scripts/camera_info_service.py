#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import yaml
import os
import rospkg
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from sensor_msgs.msg import CameraInfo

class CameraInfoService:
    def __init__(self):
        rospy.init_node('camera_info_service_node', anonymous=True)
        
        # ========== 1. 获取包路径 + 定义标定参数文件路径 ==========
        self.rpkg = rospkg.RosPack()
        self.package_path = self.rpkg.get_path('usb_camera')  # 获取usb_camera包的绝对路径
        self.calib_file = os.path.join(self.package_path, 'config', 'usb_cam_calib.yaml')  # 标定文件路径
        
        # ========== 2. 初始化CameraInfo缓存（优先加载本地config文件） ==========
        self.camera_info = CameraInfo()
        self.load_calib_params()  # 从本地yaml加载参数
        
        # ========== 3. 声明服务 + 初始化发布者 ==========
        self.service = rospy.Service('/usb_cam/set_camera_info', SetCameraInfo, self.handle_set_camera_info)
        self.camera_info_pub = rospy.Publisher('/usb_cam/camera_info', CameraInfo, queue_size=10)
        self.pub_rate = rospy.Rate(rospy.get_param('/usb_camera_node/fps', 30))  # 和摄像头帧率同步
        
        rospy.loginfo(f"Calibration file path: {self.calib_file}")
        rospy.loginfo("Service /usb_cam/set_camera_info is ready!")
        rospy.loginfo(f"Initial camera info loaded: {'Calibrated' if self.camera_info.K[0] != 0.0 else 'Default'}")

    def load_calib_params(self):
        """从本地config目录的yaml文件加载标定参数，无则初始化默认值"""
        try:
            # 读取本地yaml文件
            with open(self.calib_file, 'r') as f:
                calib_params = yaml.safe_load(f)
            
            # 组装为CameraInfo消息对象
            self.camera_info.width = calib_params['width']
            self.camera_info.height = calib_params['height']
            self.camera_info.distortion_model = calib_params['distortion_model']
            self.camera_info.K = calib_params['K']
            self.camera_info.D = calib_params['D']
            self.camera_info.R = calib_params['R']
            self.camera_info.P = calib_params['P']
            
            rospy.loginfo(f"Loaded calibration params from local file: {self.calib_file}")
        except (FileNotFoundError, KeyError):
            # 无本地文件时，初始化默认值（适配640x480）
            rospy.logwarn(f"No local calibration file found ({self.calib_file}), use default params!")
            self.camera_info.width = 640
            self.camera_info.height = 480
            self.camera_info.distortion_model = "plumb_bob"
            self.camera_info.K = [302.692266, 0.0, 289.088381, 0.0, 301.656583, 231.853136, 0.0, 0.0, 1.0]
            self.camera_info.D = [0.030932, -0.037354, -0.001384, -0.004103, 0.000000]  
            self.camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # 无旋转
            self.camera_info.P = [303.481445, 0.000000, 287.173503, 0.000000, 0.000000, 302.597321, 230.375483, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]  # 无投影偏移

    def save_calib_params(self, calib_params):
        """将标定参数保存到本地config目录的yaml文件"""
        try:
            # 写入yaml文件（缩进2，便于阅读）
            with open(self.calib_file, 'w') as f:
                yaml.dump(calib_params, f, indent=2)
            rospy.loginfo(f"Calibration params saved to local file: {self.calib_file}")
        except Exception as e:
            rospy.logerr(f"Failed to save calib params to file: {str(e)}")
            raise e

    def handle_set_camera_info(self, req):
        """处理服务请求：更新缓存 + 保存到本地yaml文件"""
        try:
            # 1. 更新本地CameraInfo缓存
            self.camera_info = req.camera_info
            
            # 2. 拆解为基础类型字典（适配yaml保存）
            calib_params = {
                "width": self.camera_info.width,
                "height": self.camera_info.height,
                "distortion_model": self.camera_info.distortion_model,
                "K": list(self.camera_info.K),
                "D": list(self.camera_info.D),
                "R": list(self.camera_info.R),
                "P": list(self.camera_info.P)
            }
            
            # 3. 保存到本地config目录
            self.save_calib_params(calib_params)
            
            # 4. 返回成功响应
            resp = SetCameraInfoResponse()
            resp.success = True
            resp.status_message = f"Camera info saved to {self.calib_file}!"
            return resp
        
        except Exception as e:
            rospy.logerr(f"Error handling set_camera_info: {str(e)}")
            resp = SetCameraInfoResponse()
            resp.success = False
            resp.status_message = f"Failed: {str(e)}"
            return resp

    def run(self):
        """主循环：持续发布camera_info话题（和摄像头帧率同步）"""
        while not rospy.is_shutdown():
            self.camera_info.header.stamp = rospy.Time.now()
            self.camera_info.header.frame_id = "usb_cam"  # 和图像frame_id一致
            self.camera_info_pub.publish(self.camera_info)
            self.pub_rate.sleep()

if __name__ == "__main__":
    # 安装rospkg（若未安装）
    try:
        import rospkg
    except ImportError:
        rospy.logerr("rospkg not found! Install with: sudo pip3 install rospkg")
        exit(1)
    
    try:
        service_node = CameraInfoService()
        service_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera info service node shutdown!")
    except Exception as e:
        rospy.logerr(f"Node startup failed: {str(e)}")
        
   
