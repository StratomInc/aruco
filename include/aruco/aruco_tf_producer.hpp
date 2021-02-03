// Copyright 2020 Stratom, Inc.

#ifndef ARUCO__ARUCO_TF_PRODUCER_HPP_
#define ARUCO__ARUCO_TF_PRODUCER_HPP_

#include <Eigen/Dense>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.h"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace aruco
{

class ArucoTFProducer : public rclcpp::Node
{
public:
  explicit ArucoTFProducer(const rclcpp::NodeOptions & options);
  ~ArucoTFProducer() {}

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_solver_sub_;
  bool arucoSolverStatusCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};
}  // namespace aruco
#endif  // ARUCO__ARUCO_TF_PRODUCER_HPP_
