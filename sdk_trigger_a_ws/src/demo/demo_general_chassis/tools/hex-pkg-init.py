# -*- coding: utf-8 -*-
pkg_name = "xpkg_demo" # 本包名称

import sys  # 检查是不是 python3
if sys.version_info.major < 3:
   print("Use python3 to run this file!")
   exit()

import os
import subprocess

# 开始本包的安装配置
# 得到ROS版本
ROS_VERSION = os.getenv('ROS_VERSION')
if (ROS_VERSION is None):
   print("Unknown ROS Verison! Please install the ros_environment package and source the /opt/ros/YOUR-ROS-VERSION/setup.bash")
   print(ROS_VERSION)
   raise RuntimeError
ROS_VERSION = int(ROS_VERSION)
# 得到本python文件的路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

'''
这里放ROS1与ROS2都需要的安装步骤(apt装包, udev等)
'''
if ROS_VERSION == 1:
   # 开始执行ROS1的安装需要
   ROS_DISTRO = os.getenv('ROS_DISTRO')
   subprocess.call(["sudo", "apt-get", "install", f"ros-{ROS_DISTRO}-teleop-twist-keyboard", "-y"])
elif ROS_VERSION == 2:
   # 开始执行ROS2的安装需要
   pass
else:
   print("Unknown ROS Verison! Please install the ros_environment package and source the /opt/ros/YOUR-ROS-VERSION/setup.bash" )
   print(ROS_VERSION)
   raise RuntimeError
