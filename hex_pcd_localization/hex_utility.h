/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#ifndef HEX_PCD_LOCALIZATION_HEX_UTILITY_H_
#define HEX_PCD_LOCALIZATION_HEX_UTILITY_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace hex {
namespace utility {

struct HexParametersFlag {
  bool auto_start;
  bool pure_lidar;
};

struct HexParametersFrame {
  std::string map;
  std::string odom;
  std::string sensor;
};

struct HexParametersInit {
  Eigen::Affine3f pose;
};

struct HexParametersNdt {
  double trans_epsilon;
  double step_size;
  double resolution;
  int32_t max_iterations;
};

struct HexParametersSource {
  double voxel_size;
  Eigen::Vector2d fov_threshold;
  Eigen::Vector2d distance_threshold;
};

struct HexParametersTarget {
  std::string map_path;
  double voxel_size;
  double pcd_size;
  double update_distance;
};

struct HexStamp {
  uint32_t sec;
  uint32_t nsec;
  // constructor
  HexStamp() : sec(0), nsec(0) {}
  HexStamp(uint32_t sec, uint64_t nsec) : sec(sec), nsec(nsec) {}
  // copy operator
  HexStamp& operator=(const HexStamp& that) {
    this->sec = that.sec;
    this->nsec = that.nsec;
    return *this;
  }
  // compare operator
  bool operator<(const HexStamp& that) const {
    if (this->sec < that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec < that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator>(const HexStamp& that) const {
    if (this->sec > that.sec) {
      return true;
    } else if ((this->sec == that.sec) && (this->nsec > that.nsec)) {
      return true;
    } else {
      return false;
    }
  }
  bool operator==(const HexStamp& that) const {
    return (this->sec == that.sec) && (this->nsec == that.nsec);
  }
  bool operator!=(const HexStamp& that) const { return !(*this == that); }
  bool operator<=(const HexStamp& that) const { return !(*this > that); }
  bool operator>=(const HexStamp& that) const { return !(*this < that); }
  // plus operator
  HexStamp operator+(double delta) const {
    if ((this->sec + this->nsec * 1e-9 + delta) < 0) {
      throw "Negative time is not allowed!";
    }

    HexStamp result = HexStamp();
    int64_t delta_sec = static_cast<int64_t>(delta);
    int64_t delta_nsec = static_cast<int64_t>((delta - delta_sec) * 1000000000);
    if (this->nsec + delta_nsec >= 1000000000) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec + 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec - 1000000000);
    } else if (this->nsec + delta_nsec < 0) {
      result.sec = static_cast<uint32_t>(static_cast<int64_t>(this->sec) +
                                         delta_sec - 1);
      result.nsec = static_cast<uint32_t>(static_cast<int64_t>(this->nsec) +
                                          delta_nsec + 1000000000);
    } else {
      result.sec =
          static_cast<uint32_t>(static_cast<int64_t>(this->sec) + delta_sec);
      result.nsec =
          static_cast<uint32_t>(static_cast<int64_t>(this->nsec) + delta_nsec);
    }
    return result;
  }
  // minus operator
  double operator-(const HexStamp& that) const {
    int64_t diff_sec =
        static_cast<int64_t>(this->sec) - static_cast<int64_t>(that.sec);
    int64_t diff_nsec =
        static_cast<int64_t>(this->nsec) - static_cast<int64_t>(that.nsec);
    double result = static_cast<double>(diff_sec) + diff_nsec * 1e-9;
    return result;
  }
  HexStamp operator-(double delta) const { return *this + (-delta); }
};

struct HexStampedTrans {
  HexStamp time;
  Eigen::Affine3f transform;
  // constructor
  HexStampedTrans() {
    this->time = HexStamp();
    this->transform = Eigen::Affine3f::Identity();
  }
  HexStampedTrans(const HexStamp& time, const Eigen::Vector3f& translation,
                  const Eigen::Quaternionf& orientation) {
    this->time = time;
    this->transform = Eigen::Affine3f(orientation);
    this->transform.translation() = translation;
  }
  // copy operator
  HexStampedTrans& operator=(const HexStampedTrans& that) {
    this->time = that.time;
    this->transform = Eigen::Affine3f(that.transform);
    return *this;
  }
};

struct HexStampedPoints {
  HexStamp time;
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  // constructor
  HexStampedPoints() {
    this->time = HexStamp();
    this->points =
        pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  }
  // copy operator
  HexStampedPoints& operator=(const HexStampedPoints& that) {
    this->time = that.time;
    this->points = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>(*that.points));
    return *this;
  }
};

}  // namespace utility
}  // namespace hex

#endif  // HEX_PCD_LOCALIZATION_HEX_UTILITY_H_
