/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-21
 ****************************************************************/

#include "hex_pcd_localization/pcd_localization.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <vector>

#include "hex_pcd_localization/data_interface/data_interface.h"

namespace hex {
namespace localization {

bool PcdLocalization::Init() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static bool first_init_flag = true;

  if (first_init_flag) {
    first_init_flag = false;

    // parameters
    kparameters_flag_ = data_interface.GetParametersFlag();
    kparameters_frame_ = data_interface.GetParametersFrame();
    kparameters_ndt_ = data_interface.GetParametersNdt();
    kparameters_source_ = data_interface.GetParametersSource();
    kparameters_target_ = data_interface.GetParametersTarget();

    // transform
    Eigen::Affine3f init_affine = data_interface.GetInit().transform;
    map_center_ = init_affine.translation();
    last_trans_ = init_affine;
    sensor_in_map_ = init_affine;
    delta_trans_ = Eigen::Affine3f::Identity();
    odom_in_map_ = Eigen::Affine3f::Identity();

    // point cloud
    map_points_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            kparameters_target_.map_path.c_str(), *map_points_) == -1) {
      first_init_flag = true;
      data_interface.Log(HexLogLevel::kError, "### Load Map Failed ###");
      return false;
    }
    map_points_ = DownSample(map_points_, kparameters_target_.voxel_size);
    data_interface.PublishMapPoints(map_points_);
    data_interface.Log(HexLogLevel::kInfo, "### Load Map Finished ###");

    if (kparameters_flag_.auto_start) {
      start_flag_ = true;
      UpdateTarget();
    } else {
      start_flag_ = false;
    }
  } else {
    start_flag_ = false;

    data_interface.Log(HexLogLevel::kInfo, "### Wait For New Init Trans ###");
  }

  return true;
}

bool PcdLocalization::Work() {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  // tf
  Eigen::Affine3f sensor_in_odom =
      data_interface.ListenFrameToSensor(kparameters_frame_.odom).transform;

  // get rough transformation
  if (data_interface.GetInitFlag()) {
    data_interface.ResetInitFlag();

    sensor_in_map_ = data_interface.GetInit().transform;
    map_center_ = sensor_in_map_.translation();
    UpdateTarget();

    if (kparameters_flag_.pure_lidar) {
      last_trans_ = sensor_in_map_;
      delta_trans_ = Eigen::Affine3f::Identity();
    }

    start_flag_ = true;

    data_interface.Log(HexLogLevel::kInfo, "### Get Init Trans ###");
  } else if (kparameters_flag_.pure_lidar) {
    sensor_in_map_ = last_trans_ * delta_trans_;
  } else {
    sensor_in_map_ = odom_in_map_ * sensor_in_odom;
  }

  if (start_flag_) {
    // update map
    if ((map_center_ - sensor_in_map_.translation()).norm() >
        kparameters_target_.update_distance) {
      // target
      map_center_ = sensor_in_map_.translation();
      UpdateTarget();
    }

    // pcd alignment
    if (data_interface.GetLidarFlag()) {
      HexStampedPoints sensor_points = data_interface.GetLidar();
      HexStampedTrans fine_sensor_in_map =
          PointsAlignment(sensor_in_map_, sensor_points);

      if (fabs(fine_sensor_in_map.time - sensor_points.time) > 1.0) {
        data_interface.Log(HexLogLevel::kError, "### Diverge ###");
        return false;
      }
      ResultProcess(fine_sensor_in_map, sensor_in_odom);

      data_interface.ResetLidarFlag();
    }
  }

  return true;
}

void PcdLocalization::UpdateTarget() {
  static DataInterface& data_interface = DataInterface::GetSingleton();

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;
  ndt_new.setTransformationEpsilon(kparameters_ndt_.trans_epsilon);
  ndt_new.setStepSize(kparameters_ndt_.step_size);
  ndt_new.setResolution(kparameters_ndt_.resolution);
  ndt_new.setMaximumIterations(kparameters_ndt_.max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr target_points = CropPointsBox(
      map_points_, kparameters_target_.pcd_size, kparameters_target_.pcd_size,
      kparameters_target_.pcd_size, map_center_);
  ndt_new.setInputTarget(target_points);
  // data_interface.PublishDebugPoints(target_points, kmap_frame_);

  ndt_register_ = ndt_new;
}

const HexStampedTrans& PcdLocalization::PointsAlignment(
    const Eigen::Affine3f& rough_sensor_in_map,
    const HexStampedPoints& sensor_points_stamped) {
  static HexStampedTrans fine_sensor_in_map;
  static DataInterface& data_interface = DataInterface::GetSingleton();

  // source
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_points = CropPointsCylinder(
      sensor_points_stamped.points, kparameters_source_.distance_threshold[0],
      kparameters_source_.distance_threshold[1],
      kparameters_source_.fov_threshold[0],
      kparameters_source_.fov_threshold[1]);
  source_points = DownSample(source_points, kparameters_source_.voxel_size);
  // data_interface.PublishDebugPoints(source_points, "base_link");

  // alignment
  fine_sensor_in_map = NdtAlignment(rough_sensor_in_map, source_points,
                                    sensor_points_stamped.time);

  // fine_sensor_in_map =
  //     DoubleIcpAlignment(rough_sensor_in_map, target_points, source_points,
  //     sensor_points_stamped.time);

  return fine_sensor_in_map;
}

void PcdLocalization::ResultProcess(const HexStampedTrans& fine_sensor_in_map,
                                    const Eigen::Affine3f& sensor_in_odom) {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  sensor_in_map_ = fine_sensor_in_map.transform;

  if (kparameters_flag_.pure_lidar) {
    delta_trans_ = last_trans_.inverse() * sensor_in_map_;
    last_trans_ = sensor_in_map_;
  }

  odom_in_map_ = sensor_in_map_ * sensor_in_odom.inverse();
  // std::cout << "map to sensor: " << std::endl
  //           << sensor_in_map_.matrix() << std::endl;
  data_interface.BroadcastMapToOdom(odom_in_map_, fine_sensor_in_map.time);
  data_interface.PublishSensorTrans(fine_sensor_in_map);
}

const HexStampedTrans& PcdLocalization::NdtAlignment(
    const Eigen::Affine3f& init_guess,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_points,
    HexStamp valid_time) {
  static DataInterface& data_interface = DataInterface::GetSingleton();
  static HexStampedTrans final_trans;

  // align
  // --- test --- //
  const auto align_start_time = std::chrono::system_clock::now();
  ndt_register_.setInputSource(source_points);
  pcl::PointCloud<pcl::PointXYZ>::Ptr align_pcd(
      new pcl::PointCloud<pcl::PointXYZ>());
  ndt_register_.align(*align_pcd, init_guess.matrix());
  const auto align_end_time = std::chrono::system_clock::now();
  double align_time = std::chrono::duration_cast<std::chrono::microseconds>(
                          align_end_time - align_start_time)
                          .count() /
                      1000.0;
  if (align_time > 100.0) {
    data_interface.Log(HexLogLevel::kError,
                       "### NDT Align Time Too Long: %f ms ###", align_time);
  }

  // check
  bool ndt_success = ndt_register_.hasConverged();
  Eigen::Affine3f ndt_result(Eigen::Affine3f::Identity());
  ndt_result.matrix() = ndt_register_.getFinalTransformation();
  if (ndt_success) {
    // final_trans = AffineToHexTrans(ndt_result, valid_time);
    final_trans.time = valid_time;
    final_trans.transform = ndt_result;
  } else {
    final_trans.time = HexStamp();
    final_trans.transform = init_guess;
  }

  return final_trans;
}

// const HexStampedTrans& PcdLocalization::DoubleIcpAlignment(
//     const Eigen::Affine3f& init_guess,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_points,
//     const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_points,
//     double valid_time) {
//   static HexStampedTrans fine_result;
//   static pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
//   icp_register; icp_register.setMaximumIterations(20);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr align_pcd(
//       new pcl::PointCloud<pcl::PointXYZ>());

//   // rough
//   pcl::PointCloud<pcl::PointXYZ>::Ptr rough_target_points =
//       DownSample(target_points, 2.0);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr rough_source_points =
//       DownSample(source_points, 0.5);
//   icp_register.setInputTarget(rough_target_points);
//   icp_register.setInputSource(rough_source_points);
//   icp_register.align(*align_pcd, init_guess.matrix());
//   bool rough_success = icp_register.hasConverged();
//   Eigen::Matrix4f rough_result = icp_register.getFinalTransformation();
//   if (!rough_success) {
//     fine_result = AffineToHexTrans(init_guess, 0.0);
//     return fine_result;
//   }

//   // fine
//   pcl::PointCloud<pcl::PointXYZ>::Ptr fine_target_points =
//       DownSample(target_points, 0.4);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr fine_source_points =
//       DownSample(source_points, 0.1);
//   icp_register.setInputTarget(fine_target_points);
//   icp_register.setInputSource(fine_source_points);
//   icp_register.align(*align_pcd, rough_result);
//   bool fine_success = icp_register.hasConverged();
//   Eigen::Affine3f align_result(Eigen::Affine3f::Identity());
//   align_result.matrix() =
//   icp_register.getFinalTransformation(); if (fine_success) {
//     fine_result = AffineToHexTrans(align_result, valid_time);
//   } else {
//     fine_result = AffineToHexTrans(align_result, 0.0);
//   }

//   return fine_result;
// }

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::DownSample(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double voxel_size) {
  static pcl::VoxelGrid<pcl::PointXYZ> voxel_down_sampler;

  pcl::PointCloud<pcl::PointXYZ>::Ptr down_sampled_pcd(
      new pcl::PointCloud<pcl::PointXYZ>());
  voxel_down_sampler.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_down_sampler.setInputCloud(raw_pcd);
  voxel_down_sampler.filter(*down_sampled_pcd);

  return down_sampled_pcd;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::CropPointsBox(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double offset_x,
    double offset_y, double offset_z, const Eigen::Vector3f& translation) {
  static pcl::CropBox<pcl::PointXYZ> crop_box_filter;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  Eigen::Vector4f min_point(translation.x() - offset_x,
                            translation.y() - offset_y,
                            translation.z() - offset_z, 1.0);
  Eigen::Vector4f max_point(translation.x() + offset_x,
                            translation.y() + offset_y,
                            translation.z() + offset_z, 1.0);
  crop_box_filter.setMin(min_point);
  crop_box_filter.setMax(max_point);
  crop_box_filter.setNegative(false);
  crop_box_filter.setInputCloud(raw_pcd);
  crop_box_filter.filter(*cropped_points);

  return cropped_points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PcdLocalization::CropPointsCylinder(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& raw_pcd, double distance_min,
    double distance_max, double angle_min, double angle_max) {
  static pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_points(
      new pcl::PointCloud<pcl::PointXYZ>());
  cropped_points->clear();

  for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = raw_pcd->begin();
       it != raw_pcd->end(); it++) {
    const pcl::PointXYZ& point = *it;
    double distance_square = point.x * point.x + point.y * point.y;
    double angle = atan2(point.y, point.x);
    if (distance_square < distance_max * distance_max &&
        distance_square > distance_min * distance_min && angle < angle_max &&
        angle > angle_min) {
      cropped_points->points.push_back(point);
    }
  }

  return cropped_points;
}

}  // namespace localization
}  // namespace hex
