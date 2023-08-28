# VINS-Fusion

## ROS2 version of VINS-Fusion.

### Notices
- code has been updated so that the vins package can be executed via ros2 run or ros2 launch
- but now it has dependency on Opencv 4.0.x

### Prerequisites
- **System**
  - Ubuntu 20.04
  - ROS2 foxy
- **Libraries**
  - OpenCV 4.0.x
  - [Ceres Solver-2.1.0](http://ceres-solver.org/installation.html) (you can refer [here](https://github.com/zinuok/VINS-Fusion#-ceres-solver-1); just edit 1.14.0 to 2.1.0 for install.)
  - [Eigen-3.3.9](https://github.com/zinuok/VINS-Fusion#-eigen-1)


### sensor setup
- camera: Intel realsense D435i
- using following shell script, you can install realsense SDK with ROS2 package.
```bash
chmod +x realsense_install.sh
bash realsense_install.sh
```
- or you can install it by apt refer to wiki

### build
```bash
cd $(PATH_TO_YOUR_ROS2_WS)/src
git clone https://github.com/huyating99/vins_mapping
cd ..
Edit Ceres_Dir in CMakeList.txt 
Edit output_path in vins_mapping/config yaml
Edit launch file in vins_mapping/vins/launch file
colcon build --symlink-install && source ./install/setup.bash && source ./install/local_setup.bash
```

### launch to test euroc dataset
```bash
ros2 launch vins $(YOUR_VINS_LAUNCH_FILE)
```

### run in every terminal
```bash
# vins
ros2 run vins vins_node $(PATH_TO_YOUR_VINS_CONFIG_FILE)

# loop_fusion
ros2 run loop_fusion loop_fusion_node $(PATH_TO_YOUR_VINS_CONFIG_FILE)

# Rviz2 visualization in vins_mapping/config
ros2 run rviz2 rviz2 -dvins_rviz_config_loop.rviz

# rosbag play 
ros2 bag play MH_01_easy.db3
```

### play bag recorded at ROS1
Unfortunately, you can't just play back the bag file recorded at ROS1. 
This is because the filesystem structure for bag file has been changed significantly.
The bag file at ROS2 needs the folder with some meta data for each bag file, which is done using following commands.
- you have to install [this pkg](https://gitlab.com/ternaris/rosbags)
```bash
pip install rosbags
```
- run
```bash
export PATH=$PATH:~/.local/bin
rosbags-convert foo.bag --dst /path/to/bar
```

### ros2bag record 
```bash
ros2 bag record -o rs_rb5_room /camera aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info /camera/color/image_raw /camera/imu /tf
```

### draw traj using evo
```bash
evo_traj tum ./rs_rb5_office_01_vio.csv --ref=./rs_rb5_office_01_vio_loop.csv -p --plot_mode=xyz -a --save_table rs_rb5_office_01
```

## Original Readme:

## 8. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).


## 9. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
