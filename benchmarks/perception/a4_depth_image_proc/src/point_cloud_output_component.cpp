#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a4_depth_image_proc/point_cloud_output_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf
{

namespace perception
{

PointCloudOutputComponent::PointCloudOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudOutputComponent", options)
{

  //Create subscriber to point cloud messages with callback function
  sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  "/benchmark/points", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(), std::bind(&PointCloudOutputComponent::pointcloudCb, this, std::placeholders::_1));

}

void PointCloudOutputComponent::pointcloudCb(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
{
  TRACEPOINT(
    robotperf_pointcloud_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*point_cloud_msg)),
    point_cloud_msg->header.stamp.nanosec,
    point_cloud_msg->header.stamp.sec);

  TRACEPOINT(
    robotperf_pointcloud_output_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*point_cloud_msg)),
    point_cloud_msg->header.stamp.nanosec,
    point_cloud_msg->header.stamp.sec);

}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PointCloudOutputComponent)
