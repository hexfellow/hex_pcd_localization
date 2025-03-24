/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_PCD_LOCALIZATION_DATA_INTERFACE_ROS1_INTERFACE_H_
#define HEX_PCD_LOCALIZATION_DATA_INTERFACE_ROS1_INTERFACE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "hex_pcd_localization/data_interface/base_interface.h"
#include "hex_pcd_localization/hex_utility.h"
#include "sensor_msgs/PointCloud2.h"

using hex::utility::HexParametersFlag;
using hex::utility::HexParametersFrame;
using hex::utility::HexParametersInit;
using hex::utility::HexParametersNdt;
using hex::utility::HexParametersSource;
using hex::utility::HexParametersTarget;
using hex::utility::HexStamp;
using hex::utility::HexStampedPoints;
using hex::utility::HexStampedTrans;

namespace hex {
namespace localization {

class DataInterface : public BaseInterface {
 public:
  static DataInterface& GetSingleton() {
    static DataInterface singleton;
    return singleton;
  }

  // Interface Handle
  void Log(HexLogLevel, const char*, ...) override;
  inline void Shutdown() override { ros::shutdown(); }
  inline bool Ok() override { return ros::ok(); }
  inline HexStamp GetTime() override {
    ros::Time time = ros::Time::now();
    return HexStamp(time.sec, time.nsec);
  }
  inline void Work() override {
    timer_->reset();
    while (ros::ok()) {
      timer_handle_();
      timer_->sleep();
    }
  }

  // Publisher Handle
  void PublishSensorTrans(const HexStampedTrans&) override;
  void PublishMapPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&) override;
  void PublishDebugPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                          const std::string&) override;

  // Initialization Handle
  void Init(int, char*[], std::string, double, void (*)()) override;
  void Deinit() override;

  // Other Handle
  void BroadcastMapToOdom(const Eigen::Affine3f&, HexStamp) override;
  const HexStampedTrans& ListenFrameToSensor(const std::string&) override;

 protected:
  // Subscriber Handle
  void InitTransHandle(const geometry_msgs::PoseWithCovarianceStampedPtr&);
  void PointCloudHandle(const sensor_msgs::PointCloud2Ptr&);

 private:
  DataInterface() = default;
  virtual ~DataInterface() = default;

  // Initialization Handle
  void ParameterInit() override;
  void VariableInit() override;
  void PublisherInit() override;
  void SubscriberInit() override;
  void TimerInit(double, void (*)()) override;

  // Node Handle
  ros::NodeHandle* nh_ptr_;
  ros::NodeHandle* nh_local_ptr_;

  // Timer Handle
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  std::unique_ptr<ros::Rate> timer_;

  // Publisher Handle
  ros::Publisher sensor_trans_pub_;
  ros::Publisher map_points_pub_;
  ros::Publisher debug_points_pub_;

  // Subscriber Handle
  ros::Subscriber init_trans_sub_;
  ros::Subscriber lidar_points_sub_;
};

}  // namespace localization
}  // namespace hex

#endif  // HEX_PCD_LOCALIZATION_DATA_INTERFACE_ROS1_INTERFACE_H_
