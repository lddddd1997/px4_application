### 一、ROS安装

* http://wiki.ros.org/cn/ROS/Installation

* 根据ubuntu版本选择安装

### 二、PX4开发环境

**以1.10版本为例：**

https://dev.px4.io/v1.10/en/setup/dev_env_linux_ubuntu.html

To install the toolchain:

1. Download [ubuntu.sh](https://github.com/PX4/Firmware/blob/v1.10.0/Tools/setup/ubuntu.sh) and [requirements.txt](https://github.com/PX4/Firmware/blob/v1.10.0/Tools/setup/requirements.txt) from the PX4 source repository (**/Tools/setup/**):
   `wget https://raw.githubusercontent.com/PX4/Firmware/v1.10.0/Tools/setup/ubuntu.sh`
   `wget https://raw.githubusercontent.com/PX4/Firmware/v1.10.0/Tools/setup/requirements.txt`

2. Run the **ubuntu.sh** with no arguments (in a bash shell) to install everything:

   ```bash
   source ubuntu.sh
   ```

**注：**需解决PX4的gazebo版本与ROS的gazebo版本的区别，即`ros-kinetic-gazebo*`的工具包与版本对应

