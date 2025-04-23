# **hex_pcd_localization**

## **Overview**

This **hex_pcd_localization** repository provides an implementation of point cloud localization based on NDT algorithm.

### **License**

This project is licensed under the terms of the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

### **Maintainer**

**[Dong Zhaorui](https://github.com/IBNBlank)**
**[Zhang Jianhuai](https://github.com/aalicecc)**

### **Supported Platform**

- [x] **x64**
- [ ] **Jetson Orin Nano**
- [x] **Jetson Orin NX**
- [ ] **Jetson AGX Orin**
- [ ] **Horizon RDK X5**

### **Supported ROS Version**

- [x] **ROS Noetic**
- [ ] **ROS Humble**

---

## **Public APIs**

### **Publish**

| Topic                | Msg Type                                    | Description                |
| -------------------- | ------------------------------------------ | --------------------------- |
| `/sensor_trans`      | `geometry_msgs/PoseWithCovarianceStamped`  | Localization result         |
| `/map_points`        | `sensor_msgs/PointCloud2`                  | Map point cloud             |
| `/debug_points`      | `sensor_msgs/PointCloud2`                  | Debug point cloud           |

### **Subscribe**

| Topic                | Msg Type                                    | Description                |
| -------------------- | ------------------------------------------ | ----------------------------|
| `/init_trans`        | `geometry_msgs/PoseWithCovarianceStamped`  | Initial pose                |
| `/point_cloud`       | `sensor_msgs/PointCloud2`                  | Sensor point cloud          |

### **Parameters**

| Name                  | Data Type             | Description                     |
| -------------------- | --------------------- | ---------------------------------|
| `flag_auto_start`    | `bool`               | Auto start flag                   |
| `flag_pure_lidar`    | `bool`               | Pure lidar mode flag              |
| `frame_map`          | `std::string`        | Map frame name                    |
| `frame_odom`         | `std::string`        | Odom frame name                   |
| `frame_sensor`       | `std::string`        | Sensor frame name                 |
| `init_position`      | `std::vector<double>` | Initial position [x,y,z]         |
| `init_orientation`   | `std::vector<double>` | Initial orientation [w,x,y,z]    |
| `ndt_trans_epsilon` | `double`             | NDT transform epsilon              |
| `ndt_step_size`     | `double`             | NDT optimization step size         |
| `ndt_resolution`    | `double`             | NDT grid resolution                |
| `ndt_max_iterations`| `int32`                | NDT maximum iterations           |
| `source_voxel_size` | `double`             | Source cloud voxel size            |
| `source_fov`        | `std::vector<double>` | Source cloud FOV range            |
| `source_distance`   | `std::vector<double>` | Source cloud distance range       |
| `target_voxel_size` | `double`             | Target cloud voxel size            |
| `target_pcd_size`   | `double`             | Target cloud crop size             |
| `target_update_distance` | `double`         | Target cloud update distance      |

---

## **Getting Started**

### **Dependencies**

- **ROS Noetic** or **ROS Humble**

### **Install**

1. Create a workspace `catkin_ws` and get into the `src`.

   ```shell
   mdkir -p catkin_ws/src
   cd catkin_ws/src
   ```

2. Clone this repo.

   ```shell
   git clone git@github.com:hexfellow/hex_pcd_localization.git
   ```

3. Go to `catkin_ws` directory and build the repo.

   ```shell
   cd ../
   catkin_make
   ```

4. Source the `setup.bash` and run the test below

   ```shell
   source devel/setup.bash --extend
   ```

### **Usage**

1. Launch the localization node:

   ```shell
   roslaunch hex_pcd_localization hex_pcd_localization.launch
   ```
