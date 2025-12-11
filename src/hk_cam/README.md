#### 海康威视工业摄像头驱动
该Demo是基于ROS1进行开发的海康工业摄像头调用驱动，并实时输出发布的ROS图像消息topic

### SDK安装
在海康工业相机官网下载SDK，注意需要根据不同的系统下载对应的SDK版本安装。
网址：https://www.hikrobotics.com/cn/machinevision/service/download?module=0

### 使用
```
mkdir -p hkcam_ws/src
unzip hk_cam.zip
cp hk_cam hkcam_ws/src -rf
cd hkcam_ws
catkin_make #编译

source devel/setup.bash
roslaunch hk_cam hk_cam.launch
```

### 相机内参标定
``` bash 
rosrun camera_calibration cameracalibrator.py --size 7x8 --square 35.2 image:=/hk/image_rect camera:=/hk --no-service-check

```
