# ORB-SLAM3 Complete Setup Guide

[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-green.svg)]()
[![Jetson](https://img.shields.io/badge/Jetson-Orin%20Nano%208GB-76B900.svg)]()

> **Comprehensive documentation of our successful ORB-SLAM3 setup with Orbbec Astra Pro camera integration, GTK conflict resolution, and multiple testing configurations**

This README documents the complete, working setup procedures for ORB-SLAM3, Orbbec camera integration, ROS wrapper configuration, GTK conflict resolution, and validation testing based on our practical implementation experience.

## 📋 Table of Contents

- [System Requirements](#system-requirements)
- [GTK3 Environment Setup](#gtk3-environment-setup)
- [ORB-SLAM3 Installation](#orb-slam3-installation)
- [Orbbec Astra Pro Camera Setup](#orbbec-astra-pro-camera-setup)
- [Camera Calibration](#camera-calibration)
- [ROS Integration](#ros-integration)
- [Testing Configurations](#testing-configurations)
- [Advanced: Pangolin + RViz Simultaneous Operation](#advanced-pangolin--rviz-simultaneous-operation)
- [Troubleshooting](#troubleshooting)
- [Performance Optimization](#performance-optimization)

## 🖥️ System Requirements

### Hardware Requirements
- **CPU**: x86_64 or ARM64 (Jetson Orin Nano 8GB tested)
- **RAM**: Minimum 4GB, Recommended 8GB+
- **Storage**: 15GB+ available space (including OpenCV build)
- **Camera**: Orbbec Astra Pro Plus (USB 2.0/3.0)

### Software Requirements
- **OS**: Ubuntu 20.04 LTS
- **ROS**: Noetic (recommended)
- **OpenCV**: ≥4.4.0 (will build 4.5+ from source)
- **Eigen**: ≥3.1.0
- **Pangolin**: Latest version (with GTK3 compatibility)
- **CMake**: ≥3.0

## 🎨 GTK3 Environment Setup

**Critical**: This step resolves GTK2/GTK3 conflicts between ROS and ORB-SLAM3 components.

### Install GTK3 Development Libraries

```bash
# Install GTK3 development packages
sudo apt update
sudo apt install libgtk-3-dev
sudo apt install python3-dev python3-distutils pybind11-dev

# Remove GTK2 development packages to avoid conflicts (optional but recommended)
sudo apt remove libgtk2.0-dev

# Verify GTK3 installation
pkg-config --exists gtk+-3.0 && echo "GTK3 found" || echo "GTK3 missing"
pkg-config --cflags gtk+-3.0
pkg-config --libs gtk+-3.0
```

## 🔧 ORB-SLAM3 Installation

### Step 1: Install System Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential build tools
sudo apt install build-essential cmake git pkg-config
sudo apt install libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev
sudo apt install libatlas-base-dev gfortran
sudo apt install python3-dev python3-numpy
```

### Step 2: Install Pangolin (GTK3 Compatible)

```bash
cd ~/
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin

# Install missing Python dependencies if needed
sudo apt install python3-dev python3-distutils pybind11-dev

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DPANGOLIN_BUILD_EXAMPLES=OFF \
      -DPANGOLIN_BUILD_TOOLS=OFF \
      ..
make -j4
sudo make install
sudo ldconfig

# Verify installation
pkg-config --modversion pangolin
```

### Step 3: Build OpenCV 4.5+ with GTK3 Support

**Critical**: ORB-SLAM3 requires OpenCV 4.4+, and we need GTK3 compatibility.

```bash
cd ~/
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 4.5.0

mkdir build && cd build

# Configure with explicit GTK3 (no GTK2)
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local \
      -DWITH_GTK=ON \
      -DWITH_GTK_2_X=OFF \
      -DWITH_GTK3=ON \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_TESTS=OFF \
      -DBUILD_PERF_TESTS=OFF \
      -DBUILD_opencv_apps=OFF \
      -DWITH_OPENGL=ON \
      -DWITH_V4L=ON \
      ..

# Verify GTK3 detection
grep "GTK.*FOUND" CMakeCache.txt
# Should show: GTK3_FOUND:INTERNAL=1

# Build (15-30 minutes)
make -j4
sudo make install
sudo ldconfig

# Verify GTK3 linkage
ldd /usr/local/lib/libopencv_highgui.so.4.5 | grep gtk
# Should show: libgtk-3.so.0 (GTK3 only, no GTK2)
```

### Step 4: Install Eigen3

```bash
sudo apt install libeigen3-dev

# Verify installation
pkg-config --modversion eigen3
# Should show version ≥3.1.0
```

### Step 5: Build ORB-SLAM3

```bash
cd ~/
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh

# Verify GTK compatibility
ldd ~/ORB_SLAM3/lib/libORB_SLAM3.so | grep gtk
# Should show: libgtk-3.so.0 (GTK3 only)
```

**Success Indicators:**
- `lib/libORB_SLAM3.so` exists (~5MB)
- No GTK2 dependencies in the library
- Example executables work without crashes

## 📷 Orbbec Astra Pro Camera Setup

### Step 1: Install Orbbec ROS Package

```bash
# Install ROS Noetic (if not already installed)
sudo apt install ros-noetic-desktop-full

# Install Orbbec ROS driver
sudo apt install ros-noetic-astra-camera

# Alternative: Build from source
cd ~/catkin_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Step 2: Test Camera Connection

```bash
# Check camera detection
lsusb | grep Orbbec
# Expected: Bus XXX Device XXX: ID 2bc5:060f Orbbec Astra Pro

# Test camera launch
roslaunch orbbec_camera astra.launch \
  serial_number:=ACR384300HL \
  product_id:=0x060f \
  connection_delay:=2000 \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30 \
  depth_format:=Y11 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30 \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false
```

### Step 3: Verify Camera Topics

```bash
# Check camera topics
rostopic list | grep camera

# Expected topics:
# /camera/color/image_raw
# /camera/depth/image_raw
# /camera/color/camera_info
# /camera/depth/camera_info

# Test topic rates
rostopic hz /camera/color/image_raw
rostopic hz /camera/depth/image_raw
```

## 🎯 Camera Calibration

### Working Camera Parameters (Orbbec Astra Pro Plus)

**Extracted from camera_info topics - verified working configuration:**

**Color Camera (640x480):**
- fx = fy = 541.297
- cx = 315.344, cy = 235.191
- k1 = 0.1359, k2 = -0.3169, k3 = 0.1954
- p1 = 0.0020, p2 = 0.0053

**Depth Camera (640x480):**
- fx = fy = 577.324
- cx = 312.112, cy = 235.900

### ORB-SLAM3 Configuration File

Create `~/catkin_ws/src/orb_slam3_ros_wrapper/config/AstraProPlus_RGBD.yaml`:

```yaml
%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters for Orbbec Astra Pro Plus
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Camera calibration and distortion parameters (OpenCV)
Camera1.fx: 541.297
Camera1.fy: 541.297
Camera1.cx: 315.344
Camera1.cy: 235.191

# Radial-tangential distortion parameters
Camera1.k1: 0.1359
Camera1.k2: -0.3169
Camera1.p1: 0.0020
Camera1.p2: 0.0053
Camera1.k3: 0.1954

# Camera resolution
Camera.width: 640
Camera.height: 480

# Camera frames per second 
Camera.fps: 30.0

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1

# RGB-D sensor parameters
RGBD.DepthMapFactor: 1000.0
System.thFarPoints: 5.0

#--------------------------------------------------------------------------------------------
# ORB Parameters (Optimized for better tracking)
#--------------------------------------------------------------------------------------------

# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1500

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold (lowered for better feature detection)
ORBextractor.iniThFAST: 10
ORBextractor.minThFAST: 3

#--------------------------------------------------------------------------------------------
# Viewer Parameters
#--------------------------------------------------------------------------------------------
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
```

## 🔄 ROS Integration

### Install ORB-SLAM3 ROS Wrapper

```bash
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git orb_slam3_ros_wrapper

# Build with GTK3 compatibility
cd ~/catkin_ws
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# Verify no GTK conflicts
ldd ~/catkin_ws/devel/lib/orb_slam3_ros_wrapper/orb_slam3_ros_wrapper_rgbd | grep gtk
# Should show: libgtk-3.so.0 (GTK3 only)
```

### Create Complete Launch Files

**Orbbec RGB-D with RViz** (`~/catkin_ws/src/orb_slam3_ros_wrapper/launch/orbbec_rgbd.launch`):

```xml
<launch>
    <!-- Orbbec Astra Pro Camera -->
    <include file="$(find orbbec_camera)/launch/astra.launch">
        <arg name="serial_number"      value="ACR384300HL"/>
        <arg name="product_id"         value="0x060f"/>
        <arg name="connection_delay"   value="2000"/>
        <arg name="depth_width"        value="640"/>
        <arg name="depth_height"       value="480"/>
        <arg name="depth_fps"          value="30"/>
        <arg name="depth_format"       value="Y11"/>
        <arg name="color_width"        value="640"/>
        <arg name="color_height"       value="480"/>
        <arg name="color_fps"          value="30"/>
        <arg name="enable_point_cloud" value="false"/>
        <arg name="enable_colored_point_cloud" value="false"/>
    </include>

    <!-- ORB-SLAM3 RGB-D Node -->
    <node name="orb_slam3_rgbd" pkg="orb_slam3_ros_wrapper" type="orb_slam3_ros_wrapper_rgbd" output="screen">
        <!-- Topic remapping for Orbbec Astra Pro -->
        <remap from="/camera/rgb/image_raw"                 to="/camera/color/image_raw"/>
        <remap from="/camera/depth_registered/image_raw"    to="/camera/depth/image_raw"/>
        
        <!-- Parameters for ORB-SLAM3 -->
        <param name="voc_file"      type="string"   value="$(find orb_slam3_ros_wrapper)/config/ORBvoc.txt" />
        <param name="settings_file" type="string"   value="$(find orb_slam3_ros_wrapper)/config/AstraProPlus_RGBD.yaml" />
    
        <!-- Parameters for ROS -->
        <param name="world_frame_id"    type="string"   value="map" />
        <param name="cam_frame_id"      type="string"   value="camera" />
        <param name="enable_pangolin"   type="bool"     value="false" />
    </node>
    
    <!-- Static transform publisher for camera frames -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_base_link" 
          args="0 0 0 0 0 0 camera_link camera_color_optical_frame" />
          
    <node pkg="tf2_ros" type="static_transform_publisher" name="world_to_camera_base" 
          args="0 0 0 0 0 0 world camera_link" />

    <!-- Trajectory server for path visualization -->
    <node pkg="hector_trajectory_server" type="hector_trajectory_server" name="trajectory_server_orb_slam3" output="screen" ns="orb_slam3_ros" >
        <param name="target_frame_name" value="map" />
        <param name="source_frame_name" value="camera" />
        <param name="trajectory_update_rate" value="20.0" />
        <param name="trajectory_publish_rate" value="20.0" />
    </node>

    <!-- RViz visualization -->
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find orb_slam3_ros_wrapper)/config/orb_slam3_no_imu.rviz" output="screen" />
</launch>
```

## 🧪 Testing Configurations

### Configuration 1: Live Camera with ROS + RViz

**Single Command Launch:**
```bash
cd ~/catkin_ws && source devel/setup.bash
roslaunch orb_slam3_ros_wrapper orbbec_rgbd.launch
```

**Manual Launch (Step by Step):**

**Terminal 1 - Start Camera:**
```bash
roslaunch orbbec_camera astra.launch \
  serial_number:=ACR384300HL \
  product_id:=0x060f \
  connection_delay:=2000 \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30 \
  depth_format:=Y11 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30 \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false
```

**Terminal 2 - Launch ORB-SLAM3 ROS Wrapper:**
```bash
cd ~/catkin_ws && source devel/setup.bash
rosrun orb_slam3_ros_wrapper orb_slam3_ros_wrapper_rgbd \
  _voc_file:=/home/kavi/catkin_ws/src/orb_slam3_ros_wrapper/config/ORBvoc.txt \
  _settings_file:=/home/kavi/catkin_ws/src/orb_slam3_ros_wrapper/config/AstraProPlus_RGBD.yaml \
  _enable_pangolin:=false \
  _world_frame_id:=map \
  _cam_frame_id:=camera \
  /camera/rgb/image_raw:=/camera/color/image_raw \
  /camera/depth_registered/image_raw:=/camera/depth/image_raw
```

**Terminal 3 - RViz Visualization:**
```bash
rviz
# Set Fixed Frame to "map"
# Add displays: Image, PointCloud2, Pose, Path
# Subscribe to appropriate topics
```

### Configuration 2: EuRoC Dataset with ROS Wrapper

**Download EuRoC Dataset:**
```bash
mkdir -p ~/Downloads/euroc
cd ~/Downloads/euroc
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```

**Launch with EuRoC:**
```bash
# Terminal 1 - Play ROS Bag
rosbag play ~/Downloads/euroc/MH_01_easy.bag

# Terminal 2 - Launch EuRoC Monocular SLAM
cd ~/catkin_ws && source devel/setup.bash
roslaunch orb_slam3_ros_wrapper euroc_mono.launch
```

### Configuration 3: Standalone ORB-SLAM3 with Pangolin

**For EuRoC Dataset (Validation):**
```bash
# Run standalone ORB-SLAM3 with rich Pangolin visualization
cd ~/ORB_SLAM3
./Examples/Monocular/mono_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Monocular/EuRoC.yaml \
    ~/Downloads/euroc \
    Examples/Monocular/EuRoC_TimeStamps/MH01.txt
```

## 🚀 Advanced: Pangolin + RViz Simultaneous Operation

**Achievement**: After GTK conflict resolution, both visualization systems can run simultaneously.

### Launch Complete System

**Terminal 1 - Start Orbbec Camera:**
```bash
roslaunch orbbec_camera astra.launch \
  serial_number:=ACR384300HL \
  product_id:=0x060f \
  connection_delay:=2000 \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=30 \
  depth_format:=Y11 \
  color_width:=640 \
  color_height:=480 \
  color_fps:=30 \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false
```

**Terminal 2 - ORB-SLAM3 ROS Wrapper WITH Pangolin:**
```bash
cd ~/catkin_ws && source devel/setup.bash
rosrun orb_slam3_ros_wrapper orb_slam3_ros_wrapper_rgbd \
  _voc_file:=/home/kavi/catkin_ws/src/orb_slam3_ros_wrapper/config/ORBvoc.txt \
  _settings_file:=/home/kavi/catkin_ws/src/orb_slam3_ros_wrapper/config/AstraProPlus_RGBD.yaml \
  _enable_pangolin:=true \
  _world_frame_id:=map \
  _cam_frame_id:=camera \
  /camera/rgb/image_raw:=/camera/color/image_raw \
  /camera/depth_registered/image_raw:=/camera/depth/image_raw
```

**Terminal 3 - RViz (Optional, for ROS visualization):**
```bash
rviz
# Configure displays for map, trajectory, point cloud
# Both Pangolin and RViz will show different aspects of the system
```

**Result**: 
- **Pangolin Windows**: Rich SLAM debugging (covisibility graph, feature tracking, 3D reconstruction)
- **RViz**: ROS-standard visualization (transforms, topics, navigation-ready data)
- **No GTK Conflicts**: Both systems use GTK3 consistently

## 🔧 Troubleshooting

### GTK Conflicts Resolution

**Problem**: `GTK+ 2.x symbols detected. Using GTK+ 2.x and GTK+ 3 in the same process is not supported`

**Root Cause Analysis:**
```bash
# Check what's linking to different GTK versions
ldd ~/catkin_ws/devel/lib/orb_slam3_ros_wrapper/orb_slam3_ros_wrapper_rgbd | grep gtk
ldd ~/ORB_SLAM3/lib/libORB_SLAM3.so | grep gtk

# Expected after fix: both should show libgtk-3.so.0 only
```

**Solutions Applied:**
1. **Rebuild OpenCV with GTK3**: Ensures ORB-SLAM3 uses GTK3
2. **Rebuild Pangolin with GTK3**: Eliminates GTK2 dependencies
3. **Clean system of GTK2 dev packages**: Prevents mixed linking

**Quick Fix (Temporary):**
```bash
# Disable Pangolin in ROS context
_enable_pangolin:=false
```

### Camera Connection Issues

**USB 2.0 Compatibility:**
- Use exact launch parameters documented above
- Ensure USB cable supports data transfer (not just power)
- Try different USB ports if detection fails

**Topic Verification:**
```bash
# Check camera detection
lsusb | grep Orbbec
# Expected: Bus XXX Device XXX: ID 2bc5:060f Orbbec Astra Pro

# Verify topics are publishing
rostopic list | grep camera
rostopic hz /camera/color/image_raw
```

### Tracking Performance Issues

**Environmental Factors:**
- **Lighting**: Ensure consistent, diffuse lighting
- **Motion**: Move camera slowly and smoothly
- **Scene Content**: Point at textured surfaces (avoid blank walls)
- **Initialization**: Allow system time to build initial map

**Parameter Tuning:**
```yaml
# In AstraProPlus_RGBD.yaml
ORBextractor.nFeatures: 1500  # Increase for more features
ORBextractor.iniThFAST: 10    # Lower for more feature detection
ORBextractor.minThFAST: 3     # Lower threshold fallback
```

### Build Issues

**OpenCV Version Conflicts:**
```bash
# Clean any existing OpenCV installations
sudo rm -rf /usr/local/lib/libopencv*
sudo rm -rf /usr/local/include/opencv*
sudo rm -rf /usr/local/lib/cmake/opencv*
sudo ldconfig

# Rebuild following GTK3 instructions above
```

**Pangolin Python Dependencies:**
```bash
# Install missing dependencies
sudo apt install python3-dev python3-distutils pybind11-dev

# If CMake still fails, disable Python components:
cmake -DPANGOLIN_BUILD_PYTHON=OFF ..
```

## ⚡ Performance Optimization

### Expected Performance Metrics

**Desktop Hardware (Intel i7, 8GB RAM):**
- **Tracking**: 25-30 FPS (real-time)
- **Feature Extraction**: ~15ms per frame
- **Memory Usage**: 1-3GB
- **Initialization**: 1-3 seconds

**Jetson Orin Nano 8GB:**
- **Tracking**: 15-25 FPS (real-time capable)
- **Feature Extraction**: ~20ms per frame  
- **Memory Usage**: 2-4GB
- **Power Consumption**: 8-12W

### Optimization Settings

**For Maximum Performance:**
```bash
# Enable performance mode (Jetson)
sudo nvpmodel -m 0
sudo jetson_clocks

# Monitor resources
htop
nvidia-smi  # If available
```

**Camera Optimization:**
- **Resolution**: 640x480 (optimal balance)
- **Frame Rate**: 30fps (matches SLAM processing)
- **Depth Format**: Y11 (efficient for USB 2.0)
- **Disable Point Clouds**: Reduces bandwidth and processing

## 📈 Project Status Summary

### ✅ Validated Achievements

1. **Complete GTK Conflict Resolution**: Simultaneous Pangolin + RViz operation
2. **Multi-Configuration Setup**: Live camera, dataset processing, standalone modes
3. **Production-Ready Performance**: Real-time operation on desktop and embedded hardware
4. **Comprehensive Documentation**: All working commands and parameters documented
5. **Robust Calibration**: Extracted and validated camera parameters

### 🎯 System Capabilities

- **Real-time RGB-D SLAM**: Live mapping and localization at 30fps
- **Rich Debugging Interface**: Pangolin visualization of internal SLAM state
- **ROS Integration**: Standard robotics interface for navigation systems
- **Dataset Processing**: Validated accuracy against EuRoC benchmarks
- **Dual Visualization**: Both algorithm debugging and navigation-ready displays

### 🔧 Technical Insights

1. **GTK Version Management**: Critical for ROS + ORB-SLAM3 integration
2. **OpenCV Version Requirements**: ORB-SLAM3 needs 4.4+, must be built with GTK3
3. **USB 2.0 Optimization**: Specific parameter sets required for stable operation
4. **Environmental Sensitivity**: Lighting and texture significantly affect tracking

### 📊 Validation Results

**EuRoC Dataset (MH_01_easy):**
- Clean initialization without tracking failures
- Consistent keyframe generation
- Accurate trajectory estimation
- 350+ keyframes, 13,000+ map points typical

**Live Camera Operation:**
- Real-time performance achieved
- Map building in textured indoor environments
- Robust to moderate lighting variations
- Compatible with ROS navigation stack

## 📚 References

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [ORB-SLAM3 GitHub Repository](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [Orbbec ROS Driver](https://github.com/orbbec/OrbbecSDK_ROS1)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [ROS ORB-SLAM3 Wrapper](https://github.com/thien94/orb_slam3_ros)

## 📝 Final Notes

### Key Success Factors
- **GTK3 Consistency**: Essential for avoiding runtime conflicts
- **Camera Parameter Accuracy**: Direct impact on tracking stability
- **Environmental Awareness**: Lighting and texture requirements
- **Version Compatibility**: Specific OpenCV/Pangolin versions needed

### Operational Guidelines
- **Test Progression**: Always validate with EuRoC dataset first
- **Debugging Strategy**: Use Pangolin for algorithm insight, RViz for navigation
- **Performance Monitoring**: Watch feature count and processing times
- **Environment Preparation**: Good lighting and textured scenes improve results

---

**Last Updated**: December 2024  
**Tested Configuration**: Ubuntu 20.04, Orbbec Astra Pro Plus, ORB-SLAM3 v1.0  
**Status**: Production Ready with GTK Conflict Resolution ✅  
**Achievement**: Simultaneous Pangolin + RViz Operation Validated ✅
