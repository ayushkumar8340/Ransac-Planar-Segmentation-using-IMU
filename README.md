# RANSAC Planar Segmentation using IMU

This repository implements a gravity-aligned RANSAC-based plane segmentation algorithm that integrates IMU data to improve plane fitting performance. The key idea is to guide the RANSAC plane hypothesis generation using the gravity direction estimated from an IMU, which stabilizes and accelerates convergence in real-world applications like SLAM or robotics.

## DEMO
The red part is the segmented plane.
![WhatsAppVideo2025-07-17at7 30 20PM-ezgif com-resize](https://github.com/user-attachments/assets/6734d226-9019-4c33-9f6a-61c1e5ecaa54)

## ✨ Features

- Plane segmentation from 3D point clouds using RANSAC
- Gravity direction estimation from IMU data
- Accelerated plane hypothesis generation using gravity alignment
- Robust to noise and partial surfaces
- No dependency on ROS – implemented with Eigen, STL, and PCL

---

## 🔧 Dependencies

- **C++14 or later**
- **Eigen3**

Install Eigen on Ubuntu:

```bash
sudo apt install libeigen3-dev
```


## Building
```bash 
mkdir build
cd build
cmake ..
make -j
```

## Running the code
- This repo has an example script for realsense camera.
- The example script segments the Z-plane

```bash
cd build
./tests/test_z_plane
```

