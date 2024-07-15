# SLAM-SpaceMapper

## Overview
Using ORB-SLAM, create a monocular camera visual SLAM system on a robot to effectively map a room. Camera feed is processed by Raspberry Pi and transferred to laptop for algorithm processing.

## Method

Before this project and researching SLAM, I wanted to create another "embedded" robot. I researched how I can get a LIDAR or pair of stereo cameras, but ultimately they were a bit too expensive. I settled on a monocular SLAM system since developping firmware/drivers for a MIPI CSI would require a hefty MPU (which I don't have the money for) and the Raspberry Pi only has 1 CSI port. 

This stack uses Scaled FAST and Oriented BRIEF for its frontend and Sliding Window Bundle Adjustment for its backend. Epipolar geometry and triangulation are used in the frontend to determine the visual odometry and 3D-feature points. 

This project would not have been possible without the slambook[https://github.com/gaoxiang12/slambook2/tree/master], it taught most concepts used in this project. 

## Prequesites

Requires G2O, OpenCV, Sophus, Eigen, OpenGL and Pangolin. Uses C++ 17.

## Running

```shell
git clone <repo>
cd <repo>
mkdir build && cd build
cmake ..
make
./app
```
