#include <rclcpp/rclcpp.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a7_pointcloud_to_laserscan/pointcloud_input_component.hpp"
#include <rclcpp/serialization.hpp>

namespace robotperf
{

namespace perception
{

PointCloudInputComponent::PointCloudInputComponent (const rclcpp::NodeOptions & options)
: rclcpp::Node("PointCloudInputComponent", options)
{

  // Get the input_topic_name parameter from the parameter server with default value "input"
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input");


  // Create a point cloud publisher
  pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(input_topic_name,
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  // Create point cloud subscriber
  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), 
    std::bind(&PointCloudInputComponent::pointCloudCb, this, std::placeholders::_1)
  );

}

size_t PointCloudInputComponent::get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg){
  //Serialize the PointCloud messages
  rclcpp::SerializedMessage serialized_data_cloud;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  const void* cloud_ptr = reinterpret_cast<const void*>(cloud_msg.get());
  cloud_serialization.serialize_message(cloud_ptr, &serialized_data_cloud);
  size_t cloud_msg_size = serialized_data_cloud.size();
  return cloud_msg_size;
}


void PointCloudInputComponent::pointCloudCb(
  sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  
  TRACEPOINT(
    robotperf_pointcloud_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*cloud_msg)),
    cloud_msg->header.stamp.nanosec,
    cloud_msg->header.stamp.sec,
    get_msg_size(cloud_msg));

  if (pub_pointcloud_->get_subscription_count() < 1) {
    return;
  }

  pub_pointcloud_->publish(*cloud_msg);

  TRACEPOINT(
    robotperf_pointcloud_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*cloud_msg)),
    cloud_msg->header.stamp.nanosec,
    cloud_msg->header.stamp.sec,
    get_msg_size(cloud_msg));
}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PointCloudInputComponent)