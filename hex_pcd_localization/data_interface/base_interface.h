/****************************************************************
 * Copyright 2025 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2025-03-14
 ****************************************************************/

#ifndef HEX_PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_
#define HEX_PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_

#include <atomic>
#include <mutex>
#include <string>

#include "hex_pcd_localization/hex_utility.h"

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

enum class HexLogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };

class BaseInterface {
 public:
  BaseInterface() : init_flag_(false), lidar_flag_(false) {}
  virtual ~BaseInterface() = default;
  BaseInterface(const BaseInterface&) = delete;
  BaseInterface& operator=(const BaseInterface&) = delete;

  // Interface Handle
  virtual void Log(HexLogLevel, const char*, ...) = 0;
  virtual void Shutdown() = 0;
  virtual bool Ok() = 0;
  virtual HexStamp GetTime() = 0;
  virtual void Work() = 0;

  // Publisher Handle
  virtual void PublishSensorTrans(const HexStampedTrans&) = 0;
  virtual void PublishMapPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&) = 0;
  virtual void PublishDebugPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr&,
                                  const std::string&) = 0;

  // Initialization Handle
  virtual void Init(int, char*[], std::string, double, void (*)()) = 0;
  virtual void Deinit() = 0;

  // Other Handle
  virtual void BroadcastMapToOdom(const Eigen::Affine3f&, HexStamp) = 0;
  virtual const HexStampedTrans& ListenFrameToSensor(const std::string&) = 0;

  // Parameter Handle
  inline const HexParametersFlag& GetParametersFlag() const {
    return kparameters_flag_;
  }
  inline const HexParametersFrame& GetParametersFrame() const {
    return kparameters_frame_;
  }
  inline const HexParametersInit& GetParametersInit() const {
    return kparameters_init_;
  }
  inline const HexParametersNdt& GetParametersNdt() const {
    return kparameters_ndt_;
  }
  inline const HexParametersSource& GetParametersSource() const {
    return kparameters_source_;
  }
  inline const HexParametersTarget& GetParametersTarget() const {
    return kparameters_target_;
  }

  // Subscriber Handle
  inline bool GetInitFlag() const { return init_flag_; }
  inline void ResetInitFlag() { init_flag_ = false; }
  inline const HexStampedTrans& GetInit() const {
    std::lock_guard<std::mutex> lock(init_mutex_);
    return init_;
  }
  inline bool GetLidarFlag() const { return lidar_flag_; }
  inline void ResetLidarFlag() { lidar_flag_ = false; }
  inline const HexStampedPoints& GetLidar() const {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return lidar_;
  }

 protected:
  // Initialization Handle
  virtual void ParameterInit() = 0;
  virtual void VariableInit() = 0;
  virtual void PublisherInit() = 0;
  virtual void SubscriberInit() = 0;
  virtual void TimerInit(double, void (*)()) = 0;

  // Time Handle
  void (*timer_handle_)();

  // Parameters
  HexParametersFlag kparameters_flag_;
  HexParametersFrame kparameters_frame_;
  HexParametersInit kparameters_init_;
  HexParametersNdt kparameters_ndt_;
  HexParametersSource kparameters_source_;
  HexParametersTarget kparameters_target_;

  // Variables
  std::atomic<bool> init_flag_;
  mutable std::mutex init_mutex_;
  HexStampedTrans init_;
  std::atomic<bool> lidar_flag_;
  mutable std::mutex lidar_mutex_;
  HexStampedPoints lidar_;
};

}  // namespace localization
}  // namespace hex

#endif  // HEX_PCD_LOCALIZATION_DATA_INTERFACE_BASE_INTERFACE_H_
