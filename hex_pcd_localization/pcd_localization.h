/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_PCD_LOCALIZATION_PCD_LOCALIZATION_H_
#define HEX_PCD_LOCALIZATION_PCD_LOCALIZATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <vector>

#include "hex_pcd_localization/hex_utility.h"

using hex::utility::HexParametersFlag;
using hex::utility::HexParametersFrame;
using hex::utility::HexParametersNdt;
using hex::utility::HexParametersSource;
using hex::utility::HexParametersTarget;
using hex::utility::HexStamp;
using hex::utility::HexStampedPoints;
using hex::utility::HexStampedTrans;

namespace hex {
namespace localization {

enum class SystemState { kMove = 0, kStart, kFinish, kCharge };

class PcdLocalization {
 public:
  static PcdLocalization& GetSingleton() {
    static PcdLocalization singleton;
    return singleton;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  PcdLocalization() = default;
  virtual ~PcdLocalization() = default;

  // Work Handle
  void UpdateTarget();
  const HexStampedTrans& PointsAlignment(const Eigen::Affine3f&,
                                         const HexStampedPoints&);
  void ResultProcess(const HexStampedTrans&, const Eigen::Affine3f&);

  // Help Handle
  const HexStampedTrans& NdtAlignment(
      const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
      HexStamp);
  // const HexStampedTrans& DoubleIcpAlignment(
  //     const Eigen::Affine3f&, const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
  //     const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr DownSample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsBox(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      const Eigen::Vector3f&);
  pcl::PointCloud<pcl::PointXYZ>::Ptr CropPointsCylinder(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr&, double, double, double,
      double);

  // Parameters
  HexParametersFlag kparameters_flag_;
  HexParametersFrame kparameters_frame_;
  HexParametersNdt kparameters_ndt_;
  HexParametersSource kparameters_source_;
  HexParametersTarget kparameters_target_;

  // Variables
  bool start_flag_;
  Eigen::Vector3f map_center_;
  Eigen::Affine3f last_trans_;
  Eigen::Affine3f delta_trans_;
  Eigen::Affine3f sensor_in_map_;
  Eigen::Affine3f odom_in_map_;
  Eigen::Affine3f rough_sensor_in_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_register_;
};

}  // namespace localization
}  // namespace hex

#endif  // HEX_PCD_LOCALIZATION_PCD_LOCALIZATION_H_
