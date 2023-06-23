#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
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

  // Get the output_topic_name parameter from the parameter server with default value "input"
  std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name", "/benchmark/points");

  //Create subscriber to point cloud messages with callback function
  sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
  output_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(), std::bind(&PointCloudOutputComponent::pointcloudCb, this, std::placeholders::_1));

}

size_t PointCloudOutputComponent::get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_msg){
  //Serialize the PointCloud2 messages
  rclcpp::SerializedMessage serialized_data_point_cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_cloud_serialization;
  const void* point_cloud_ptr = reinterpret_cast<const void*>(point_cloud_msg.get());
  point_cloud_serialization.serialize_message(point_cloud_ptr, &serialized_data_point_cloud);
  size_t point_cloud_msg_size = serialized_data_point_cloud.size();
  return point_cloud_msg_size;
}

void PointCloudOutputComponent::pointcloudCb(
  const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
{
  TRACEPOINT(
    robotperf_pointcloud_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*point_cloud_msg)),
    point_cloud_msg->header.stamp.nanosec,
    point_cloud_msg->header.stamp.sec,
    get_msg_size(point_cloud_msg));

  TRACEPOINT(
    robotperf_pointcloud_output_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*point_cloud_msg)),
    point_cloud_msg->header.stamp.nanosec,
    point_cloud_msg->header.stamp.sec,
    get_msg_size(point_cloud_msg));

}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PointCloudOutputComponent)
