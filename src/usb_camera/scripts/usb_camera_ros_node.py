#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class UsbCameraROSNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('usb_camera_node', anonymous=True)
        
        # ========== 从ROS参数服务器获取参数（可通过launch文件指定） ==========
        self.video_index = rospy.get_param('~video_index', 4)  # 摄像头索引，默认4
        self.width = rospy.get_param('~width', 640)            # 宽度，默认640
        self.height = rospy.get_param('~height', 480)          # 高度，默认480
        self.fps = rospy.get_param('~fps', 30)                 # 帧率，默认30
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')  # 发布话题
        
        # 初始化CVBridge（转换OpenCV图像到ROS消息）
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.cap = cv2.VideoCapture(self.video_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            rospy.logerr("Failed to open camera! Check video index or camera connection.")  # 修正：logerr而非ERROR
            raise rospy.ROSException("Camera open failed")
        
        # 强制指定MJPG格式（解决帧率低问题）
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # 设置分辨率和帧率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 打印实际生效参数
        self.print_actual_params()
        
        # 初始化图像发布者
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        
        # 帧率统计变量
        self.frame_count = 0
        self.start_time = time.time()
        
        # 节点关闭时的清理函数
        rospy.on_shutdown(self.cleanup)

    def print_actual_params(self):
        """打印摄像头实际生效的参数"""
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        # 解析FourCC格式
        fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
        fourcc_int = int(fourcc)
        fourcc_str = chr((fourcc_int >> 0) & 0xFF) + chr((fourcc_int >> 8) & 0xFF) + chr((fourcc_int >> 16) & 0xFF) + chr((fourcc_int >> 24) & 0xFF)
        
        rospy.loginfo("="*50)
        rospy.loginfo(f"Camera actual parameters:")
        rospy.loginfo(f"Resolution: {actual_width}x{actual_height}")
        rospy.loginfo(f"Set FPS: {self.fps} | Actual FPS: {actual_fps}")
        rospy.loginfo(f"Format: {fourcc_str}")
        rospy.loginfo(f"Publish topic: {self.image_topic}")
        rospy.loginfo("Press Ctrl+C to quit")
        rospy.loginfo("="*50)

    def run(self):
        """节点主循环"""
        rate = rospy.Rate(self.fps)  # 按设定帧率循环
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Failed to read frame from camera!")
                rate.sleep()
                continue
            
            # 实时统计实际帧率（改用print实现end='\r'）
            self.frame_count += 1
            elapsed_time = time.time() - self.start_time
            if elapsed_time >= 1.0:
                real_fps = self.frame_count / elapsed_time
                # 改用print，保留换行覆盖效果，不触发ROS日志参数错误
                print(f"Real-time FPS: {real_fps:.1f}fps", end='\r', flush=True)
                self.frame_count = 0
                self.start_time = time.time()
            
            # 将OpenCV图像转为ROS Image消息并发布
            try:
                # OpenCV默认BGR，转为ROS的RGB格式
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                ros_image.header.stamp = rospy.Time.now()  # 添加时间戳
                self.image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(f"CVBridge error: {e}")
            
            # 按设定帧率休眠
            rate.sleep()

    def cleanup(self):
        """节点关闭时释放资源"""
        rospy.loginfo("Shutting down camera node...")
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        camera_node = UsbCameraROSNode()
        camera_node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Node error: {e}")  # 修正：logerr而非直接打印