#ifndef ROBOTPERF_POINT_CLOUD_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_POINT_CLOUD_OUTPUT_COMPONENT_HPP_

#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robotperf
{

namespace perception
{

class PointCloudOutputComponent
  : public rclcpp::Node
{
public:
  explicit PointCloudOutputComponent(const rclcpp::NodeOptions &);

protected:
  // Create subscriber to Disparity image with callback function
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;

  size_t get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_msg);

  void pointcloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_POINT_CLOUD_OUTPUT_COMPONENT_HPP_