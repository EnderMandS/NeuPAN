# ROS packages to support NeuPan

The packages is the example of Unitree GO2 and Manifold Odin

## Packages

### fish2pinhole

Odin fisheye image undistort:
``` shell
roslaunch fish2pinhole undistort.launch
```

Odin point cloud FOV crop and denoise:
``` shell
roslaunch fish2pinhole cloud_crop.launch
```

###  pc2scan

Covert point cloud into scan using `pointcloud_to_laserscan` package

``` shell
roslaunch pc2scan pc2scan.launch
```

This package also contain the tf of `base_link` and `lidar`

### unitree_control

This package convert ROS topic `cmd_vel` to Unitree SDK

``` shell
rosrun unitree_control unitree_vel_controller
```

### neupan_global_planner

This package read global map, subscrible navigation goal and convert it into `nav_msgs::Path` for NeuPan


