# WheelTech USB Camera ROS Driver
作者：Komasa Qi

## 参数标定：
打印一个A3大小的棋盘格标定板，然后黏贴在一个平板上，保证平面度。然后我们启动下面的程序进行标定。需要注意的2个参数是`--size 9x6`和`--square 0.039`，这2个参数分别表示棋盘格的角点多少和每个格子的边长。如果你的棋盘格大小不是9x6，或者格子边长不是39mm，需要根据实际情况修改这2个参数。格子宽度请以实际测量为准。
``` bash
rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.039 image:=/usb_cam/image_raw camera:=/usb_cam 

```

所有x y size skew都变成绿色以后，calibration按钮才会变绿，点击calibration按钮，会进行标定（计算可能需要一分钟），标定期间画面是静止的。标定好以后画面恢复活动，然后我们点击最下面的commit按钮，相机的参数服务器会将标定参数保存在本地文件（usb_camera本ROS包中的config文件），标定结束。