# FAST-Calib ROS2 Version

Based on [engine1wu](https://github.com/hku-mars/FAST-Calib/issues/35), the ROS1 FAST-Calib project has been ported to ROS2. Tested only on Ubuntu 22.04 Humble.
## Usage
### Parameter configuration
Test data is provided under `calib_data/mid360_11`: the `mid360/11` point cloud from the [sample data](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/Eq_k_4Mf_11Eggg4a5lbRzgBHwd0EivtCJd2ExtcNlu1FA?e=vjm4gH), cropped and converted into the ROS2 format. To run this test data, only the path-related parameters in `config/qr_params.yaml` need to be updated, where `bag_path` points to the folder of the *ros2 bag PointCloud2*.
### Launch the node
```bash
ros2 launch fast_calib calib.launch.py
```


# FAST-Calib
FAST-Calib is an automatic target-based extrinsic calibration tool for LiDAR-camera systems (eg., [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)). 

**Key highlights include:** 

1. Support solid-state and mechanical LiDAR.
2. No need for any initial extrinsic parameters.
3. Achieve highly accurate calibration results **in just 2 seconds**.

**In short, it makes extrinsic calibration as simple as intrinsic calibration.**

📬 For further assistance or inquiries, please feel free to contact Chunran Zheng at zhengcr@connect.hku.hk.

<p align="center">
  <img src="./pics/calib.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Example of circle extraction from Mid360 point cloud | Right: Point cloud colored with calibrated extrinsic.</font>
</p>

## 1. Prerequisites
PCL>=1.8, OpenCV>=4.0.

## 2. Run our examples
1. Prepare the static acquisition data in `calib_data` folder (see [sample data](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/Eq_k_4Mf_11Eggg4a5lbRzgBHwd0EivtCJd2ExtcNlu1FA?e=vjm4gH) from Mid360, Avia and Ouster):
- rosbag containing point cloud messages
- corresponding image

2. Run the calibration process:
```bash
roslaunch fast_calib calib.launch
```

## 3. Run on your own sensor suite
1. Customize the calibration target in the image below.
2. Record data to rosbag.
3. Provide the instrinsic matrix in `qr_params.yaml`.
4. Set distance filter in `qr_params.yaml` for board point cloud (extra points are acceptable).
5. Calibrate now!

<p align="center">
  <img src="./pics/calibration_target.jpg" width="100%">
  <font color=#a0a0a0 size=2>Left: Actual calibration target | Right: Technical drawing with annotated dimensions.</font>
</p>

## 4. Appendix
Related article is coming soon...

The calibration target design is based on the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration).

For further details on the algorithm workflow, see [this document](https://github.com/xuankuzcr/FAST-Calib/blob/main/workflow.md).
## 5. Acknowledgments

Special thanks to [Jiaming Xu](https://github.com/Xujiaming1) for his support, [Haotian Li](https://github.com/luo-xue) for the equipment, and the [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) algorithm.