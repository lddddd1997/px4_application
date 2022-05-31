# px4_application
固件版本：`1.11.3`

## 仿真说明
* 准备

  * 准备`px4开发环境`，`px4_application`，`yolov5`等环境

  * 参考**px4_application/simulation**路径下的README.md配置代码运行所需的gazebo模型
  * 在**px4_application/config/detection_config/door_detection.yaml**中配置相机内参文件路径
  * 将**px4_application/scripts/pynodes/rospy_yolo_detector_sub_topic.py**赋予可执行权限，并且在文件中更改yolov5的路径以及模型文件路径，模型文件及数据集在`https://github.com/lddddd1997/uav_datasets`地址

* 编译

```
catkin_make
```
* 运行

```
cd */px4_application/script/simulation
bash sim_*.sh
```
* 起飞
  * 在QGC地面站中起飞并进入offboard模式

## 实验说明
* 准备
  * 在机载电脑上准备`px4_application`，`yolov5`，`tensorrt`，`ros-*-realsense2-camera`等环境
* 编译

```
catkin_make
```
* 运行

```
cd */px4_application/script/reality
bash *.sh
```
* 起飞
  * 起飞后进入offboard模式