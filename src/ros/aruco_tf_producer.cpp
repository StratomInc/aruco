// Copyright 2020 Stratom, Inc.

#include "aruco/aruco_tf_producer.hpp"

#include <memory>
#include "tf2_ros/create_timer_ros.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace aruco
{
// Added default constructor to allow for testing
ArucoTFProducer::ArucoTFProducer(const rclcpp::NodeOptions & options)
: Node("aruco_tf_producer", options)
{
  // SMM Observer TF Broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // PnP Solver Subscriber
  using namespace std::placeholders;
  aruco_solver_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "aruco_pose_solution", rclcpp::SensorDataQoS(),
    std::bind(&ArucoTFProducer::arucoSolverStatusCallback, this, _1));

  // TF Listener Stuff
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

bool ArucoTFProducer::arucoSolverStatusCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_DEBUG_STREAM(this->get_logger(), "IN ARUCO CALLBACK IN SMM TF PRODUCER");

  // For debug purposes publish raw tf from aruco solution
  geometry_msgs::msg::TransformStamped aruco_raw;
  aruco_raw.transform.translation.x = msg->pose.position.x;
  aruco_raw.transform.translation.y = msg->pose.position.y;
  aruco_raw.transform.translation.z = msg->pose.position.z;
  aruco_raw.transform.rotation.x = msg->pose.orientation.x;
  aruco_raw.transform.rotation.y = msg->pose.orientation.y;
  aruco_raw.transform.rotation.z = msg->pose.orientation.z;
  aruco_raw.transform.rotation.w = msg->pose.orientation.w;

  Eigen::Affine3d aruco_eigen = tf2::transformToEigen(aruco_raw);
  aruco_eigen = aruco_eigen.inverse(Eigen::Affine);
  geometry_msgs::msg::TransformStamped aruco_observed = tf2::eigenToTransform(aruco_eigen);
  aruco_observed.header.frame_id = "ar3p_camera_link_optical";

  // PnP Solution in World Frame
  tf_buffer_->transform(aruco_observed, aruco_observed, "world", tf2::durationFromSec(1.5));

  // Aruco Observed Frame
  aruco_observed.header.frame_id = "world";
  aruco_observed.child_frame_id = "aruco_observed_smm";
  aruco_observed.header.stamp = this->now();
  tf_broadcaster_->sendTransform(aruco_observed);

  return true;
}

}  // namespace aruco
RCLCPP_COMPONENTS_REGISTER_NODE(aruco::ArucoTFProducer)
