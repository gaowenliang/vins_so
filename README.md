# VINS-OS
## A Robust and Versatile Visual-Inertial State Estimator support Omnidirectional Camera and/or Stereo Camera

VINS-OS is a real-time SLAM framework for **Omnidirectional and/or Stereo Visual-Inertial Systems**, modified from [`VINS-MONO`](https://github.com/HKUST-Aerial-Robotics/VINS-Mono "VINS-MONO"). 
Same as `VINS-MONO`, it uses an optimization-based sliding window formulation for providing high-accuracy visual-inertial odometry. 
It features efficient IMU pre-integration with bias correction, automatic estimator initialization, online extrinsic calibration, stereo extrinsic self-calibration, failure detection and recovery. 
Remove the loop detection and global pose graph optimization of `VINS-MONO`.
VINS-OS is primarily designed for state estimation and feedback control of autonomous drones equrped with a dual-fisheye omnidirectional stereo vison system. This code runs on **Linux**, and is fully integrated with **ROS**.

**Authors:** [Wenlaing GAO](https://gaowenliang.github.io), [Tong Qin](https://github.com/qintony), [Peiliang Li](https://github.com/PeiliangLi), [Zhenfei Yang](https://github.com/dvorak0), and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from the [HUKST Aerial Robotics Group](http://uav.ust.hk/)


**Videos:**

<a href="https://www.youtube.com/embed/mv_9snb_bKs" target="_blank"><img src="http://img.youtube.com/vi/mv_9snb_bKs/0.jpg" 
alt="euroc" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/g_wN0Nt0VAU" target="_blank"><img src="http://img.youtube.com/vi/g_wN0Nt0VAU/0.jpg" 
alt="indoor_outdoor" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/I4txdvGhT6I" target="_blank"><img src="http://img.youtube.com/vi/I4txdvGhT6I/0.jpg" 
alt="AR_demo" width="240" height="180" border="10" /></a>

EuRoC dataset;                  Indoor and outdoor performance;                         AR application;

<a href="https://www.youtube.com/embed/2zE84HqT0es" target="_blank"><img src="http://img.youtube.com/vi/2zE84HqT0es/0.jpg" 
alt="MAV platform" width="240" height="180" border="10" /></a>
<a href="https://www.youtube.com/embed/CI01qbPWlYY" target="_blank"><img src="http://img.youtube.com/vi/CI01qbPWlYY/0.jpg" 
alt="Mobile platform" width="240" height="180" border="10" /></a>

 MAV application;               Mobile implementation (Video link for mainland China friends: [Video1](http://www.bilibili.com/video/av10813254/) [Video2](http://www.bilibili.com/video/av10813205/) [Video3](http://www.bilibili.com/video/av10813089/) [Video4](http://www.bilibili.com/video/av10813325/) [Video5](http://www.bilibili.com/video/av10813030/))

**Related Papers**
* **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**, Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen [techincal report](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper/tro_technical_report.pdf) 
* **Autonomous Aerial Navigation Using Monocular Visual-Inertial Fusion**, Yi Lin, Fei Gao, Tong Qin, Wenliang Gao, Tianbo Liu, William Wu, Zhenfei Yang, Shaojie Shen (***JFR*** accepted) [pdf](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/support_files/paper/jfr2017yi.pdf)  

*If you use VINS-OS for your academic research, please cite at least one of our related papers.*

## Build

### Prerequisites

### **ROS**

ROS Kinetic on Ubuntu 16.04: [ROS Installation](http://wiki.ros.org/indigo/Installation/Ubuntu).

Additional ROS pacakge requirements: [`cv-bridge`](http://wiki.ros.org/cv_bridge "cv_bridge"), [`tf`](http://wiki.ros.org/tf "tf"), [`message-filters`](http://wiki.ros.org/message_filters "message-filters"), [`image-transport`](http://wiki.ros.org/image_transport "image_transport"), [`opencv3`](http://wiki.ros.org/opencv3 "opencv3")

Other ROS pacakge requisites: special version [`camera_model`](https://github.com/gaowenliang/camera_model "camera_model"), [`code_utils`](https://github.com/gaowenliang/code_utils "code_utils").

### **Ceres Solver**
Follow [`Ceres-Solver Installation`](http://ceres-solver.org/installation.html "Ceres-Solver"), remember to **make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.2.0, Eigen 3.2.0) 

### Build VINS-OS

Clone the repository and catkin_make:
```
    cd TO-YOUR-ROS-CATKIN-WORKSPACE
    git clone git@github.com:gaowenliang/vins_os.git
    cd ../
    catkin_make
```

## Run with your device 

**DO NOT** start with a rolling shutter camera and unsync IMU (such as DJI M100 + Logitech web camera) at beginning. 

## Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qin@connect.ust.hk> or Peiliang LI <pliap@connect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojie@ust.hk>

