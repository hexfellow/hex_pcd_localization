/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-11-20
 ****************************************************************/

#include "hex_pcd_localization/data_interface/ros1_interface.h"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/TransformStamped.h"

namespace hex {
namespace localization {

void DataInterface::Log(HexLogLevel level, const char* format, ...) {
  char* buffer;

  va_list args;
  va_start(args, format);
  int32_t len = vasprintf(&buffer, format, args);
  va_end(args);

  if (len < 0) {
    ROS_FATAL("### Wrong Log Message ###");
    return;
  }

  switch (level) {
    case HexLogLevel::kDebug: {
      ROS_DEBUG("%s", buffer);
      break;
    }
    case HexLogLevel::kInfo: {
      ROS_INFO("%s", buffer);
      break;
    }
    case HexLogLevel::kWarn: {
      ROS_WARN("%s", buffer);
      break;
    }
    case HexLogLevel::kError: {
      ROS_ERROR("%s", buffer);
      break;
    }
    case HexLogLevel::kFatal: {
      ROS_FATAL("%s", buffer);
      break;
    }
    default: {
      ROS_FATAL("### Wrong Log Level ###");
      ROS_FATAL("%s", buffer);
      break;
    }
  }

  free(buffer);
}

void DataInterface::Init(int argc, char* argv[], std::string name,
                         double period, void (*handle)()) {
  ros::init(argc, argv, name);
  static ros::NodeHandle nh;
  static ros::NodeHandle nh_local("~");
  nh_ptr_ = &nh;
  nh_local_ptr_ = &nh_local;

  ParameterInit();
  VariableInit();
  PublisherInit();
  SubscriberInit();
  TimerInit(period, handle);

  Log(HexLogLevel::kInfo,
      "\033[1;32m %s: ### data interface init finish ### \033[0m", name.data());
}

void DataInterface::Deinit() {
  spinner_->stop();
  ros::shutdown();
  spinner_.reset();
  timer_.reset();
}

void DataInterface::ParameterInit() {
  // flag parameters
  nh_local_ptr_->param<bool>("flag_auto_start", kparameters_flag_.auto_start,
                             false);
  nh_local_ptr_->param<bool>("flag_pure_lidar", kparameters_flag_.pure_lidar,
                             false);

  // frame parameters
  nh_local_ptr_->param<std::string>("frame_map", kparameters_frame_.map, "map");
  nh_local_ptr_->param<std::string>("frame_odom", kparameters_frame_.odom,
                                    "odom");
  nh_local_ptr_->param<std::string>("frame_sensor", kparameters_frame_.sensor,
                                    "lidar");

  // init parameters
  std::vector<double> param_init_position;
  std::vector<double> param_init_orientation;
  nh_local_ptr_->param<std::vector<double>>(
      "init_position", param_init_position,
      std::vector<double>({0.0, 0.0, 0.0}));
  nh_local_ptr_->param<std::vector<double>>(
      "init_orientation", param_init_orientation,
      std::vector<double>({1.0, 0.0, 0.0, 0.0}));
  kparameters_init_.pose.matrix().block<3, 1>(0, 3) =
      Eigen::Vector3f(param_init_position.at(0), param_init_position.at(1),
                      param_init_position.at(2));
  kparameters_init_.pose.matrix().block<3, 3>(0, 0) =
      Eigen::Quaternionf(
          param_init_orientation.at(0), param_init_orientation.at(1),
          param_init_orientation.at(2), param_init_orientation.at(3))
          .toRotationMatrix();

  // ndt parameters
  nh_local_ptr_->param<double>("ndt_trans_epsilon",
                               kparameters_ndt_.trans_epsilon, 0.05);
  nh_local_ptr_->param<double>("ndt_step_size", kparameters_ndt_.step_size,
                               0.1);
  nh_local_ptr_->param<double>("ndt_resolution", kparameters_ndt_.resolution,
                               2.0);
  nh_local_ptr_->param<int32_t>("ndt_max_iterations",
                                kparameters_ndt_.max_iterations, 30);

  // source parameters
  std::vector<double> param_source_fov;
  std::vector<double> param_source_distance;
  nh_local_ptr_->param<double>("source_voxel_size",
                               kparameters_source_.voxel_size, 0.5);
  nh_local_ptr_->param<std::vector<double>>("source_fov", param_source_fov,
                                            std::vector<double>({M_PI, M_PI}));
  nh_local_ptr_->param<std::vector<double>>("source_distance",
                                            param_source_distance,
                                            std::vector<double>({1.0, 50.0}));
  kparameters_source_.fov_threshold =
      Eigen::Vector2d(param_source_fov.at(0), param_source_fov.at(1));
  kparameters_source_.distance_threshold =
      Eigen::Vector2d(param_source_distance.at(0), param_source_distance.at(1));

  // target parameters
  nh_local_ptr_->param<std::string>("target_map_path",
                                    kparameters_target_.map_path, "");
  nh_local_ptr_->param<double>("target_voxel_size",
                               kparameters_target_.voxel_size, 0.5);
  nh_local_ptr_->param<double>("target_pcd_size", kparameters_target_.pcd_size,
                               100.0);
  nh_local_ptr_->param<double>("target_update_distance",
                               kparameters_target_.update_distance, 45.0);
}

void DataInterface::VariableInit() {
  init_flag_ = false;
  init_ = HexStampedTrans();

  lidar_flag_ = false;
  lidar_ = HexStampedPoints();
}

void DataInterface::PublisherInit() {
  sensor_trans_pub_ =
      nh_ptr_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "sensor_trans", 1);
  map_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("map_points", 1, true);
  debug_points_pub_ =
      nh_ptr_->advertise<sensor_msgs::PointCloud2>("debug_points", 1);
}

void DataInterface::SubscriberInit() {
  init_trans_sub_ = nh_ptr_->subscribe("init_trans", 1,
                                       &DataInterface::InitTransHandle, this);
  lidar_points_sub_ = nh_ptr_->subscribe(
      "lidar_points", 1, &DataInterface::PointCloudHandle, this);
}

void DataInterface::TimerInit(double period, void (*handle)()) {
  // spin thread
  spinner_ = std::make_unique<ros::AsyncSpinner>(1);
  spinner_->start();

  // work thread
  timer_handle_ = handle;
  timer_ = std::unique_ptr<ros::Rate>(new ros::Rate(1000.0 / period));
}

void DataInterface::PublishSensorTrans(const HexStampedTrans& sensor_trans) {
  geometry_msgs::PoseWithCovarianceStampedPtr sensor_trans_ptr(
      new geometry_msgs::PoseWithCovarianceStamped);

  sensor_trans_ptr->header.stamp = ros::Time::now();
  sensor_trans_ptr->header.frame_id = kparameters_frame_.map;

  Eigen::Vector3f translation(sensor_trans.transform.translation());
  sensor_trans_ptr->pose.pose.position.x = translation.x();
  sensor_trans_ptr->pose.pose.position.y = translation.y();
  sensor_trans_ptr->pose.pose.position.z = translation.z();

  Eigen::Quaternionf orientation(sensor_trans.transform.rotation());
  sensor_trans_ptr->pose.pose.orientation.w = orientation.w();
  sensor_trans_ptr->pose.pose.orientation.x = orientation.x();
  sensor_trans_ptr->pose.pose.orientation.y = orientation.y();
  sensor_trans_ptr->pose.pose.orientation.z = orientation.z();

  sensor_trans_pub_.publish(sensor_trans_ptr);
}

void DataInterface::PublishMapPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& map_points) {
  sensor_msgs::PointCloud2Ptr map_points_ptr(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*map_points, *map_points_ptr);
  map_points_ptr->header.stamp = ros::Time::now();
  map_points_ptr->header.frame_id = kparameters_frame_.map;

  map_points_pub_.publish(map_points_ptr);
}

void DataInterface::PublishDebugPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& debug_points,
    const std::string& frame) {
  sensor_msgs::PointCloud2Ptr debug_points_ptr(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*debug_points, *debug_points_ptr);
  debug_points_ptr->header.stamp = ros::Time::now();
  debug_points_ptr->header.frame_id = frame;

  debug_points_pub_.publish(debug_points_ptr);
}

void DataInterface::BroadcastMapToOdom(const Eigen::Affine3f& trans,
                                       HexStamp time) {
  static tf2_ros::TransformBroadcaster tf_broadcaster;

  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = kparameters_frame_.map;
  transform.child_frame_id = kparameters_frame_.odom;
  transform.header.stamp = ros::Time(time.sec, time.nsec);
  transform.transform.translation.x = trans.translation().x();
  transform.transform.translation.y = trans.translation().y();
  transform.transform.translation.z = trans.translation().z();
  Eigen::Quaternionf quat = Eigen::Quaternionf(trans.rotation());
  transform.transform.rotation.w = quat.w();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();

  tf_broadcaster.sendTransform(transform);
}

const HexStampedTrans& DataInterface::ListenFrameToSensor(
    const std::string& frame) {
  static tf2_ros::Buffer tf2_buffer;
  static tf2_ros::TransformListener tf2_listener(tf2_buffer);
  static HexStampedTrans sensor_trans(HexStamp(), Eigen::Vector3f(0, 0, 0),
                                      Eigen::Quaternionf(1, 0, 0, 0));

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf2_buffer.lookupTransform(frame, kparameters_frame_.sensor,
                                           ros::Time(0));
    ros::Time time = transform.header.stamp;
    sensor_trans.time = HexStamp(time.sec, time.nsec);

    sensor_trans.transform.matrix().block<3, 1>(0, 3) = Eigen::Vector3f(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z);
    sensor_trans.transform.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z)
            .toRotationMatrix();
  } catch (tf2::TransformException& ex) {
    Log(HexLogLevel::kError, "%s", ex.what());
    ros::Time time = transform.header.stamp;
    sensor_trans.time = HexStamp(time.sec, time.nsec);

    sensor_trans.transform.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3f(0.0, 0.0, 0.0);
    sensor_trans.transform.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0).toRotationMatrix();
  }

  return sensor_trans;
}

void DataInterface::InitTransHandle(
    const geometry_msgs::PoseWithCovarianceStampedPtr& msg) {
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.2) {
    std::lock_guard<std::mutex> lock(init_mutex_);

    init_.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    init_.transform.matrix().block<3, 1>(0, 3) =
        Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    init_.transform.matrix().block<3, 3>(0, 0) =
        Eigen::Quaternionf(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
            .toRotationMatrix();

    init_flag_ = true;
  }
}

void DataInterface::PointCloudHandle(const sensor_msgs::PointCloud2Ptr& msg) {
  if (ros::Time::now().toSec() - msg->header.stamp.toSec() < 0.2 &&
      !lidar_flag_) {
    lidar_.time = HexStamp(msg->header.stamp.sec, msg->header.stamp.nsec);
    lidar_.points->clear();
    pcl::fromROSMsg(*msg, *lidar_.points);

    lidar_flag_ = true;
  }
}

}  // namespace localization
}  // namespace hex
