#ifndef ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_
#define ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robotperf
{

namespace perception
{

class PointCloudInputComponent
  : public rclcpp::Node
{
public:
  explicit PointCloudInputComponent(const rclcpp::NodeOptions &);

protected:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;

  size_t get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);

  void pointCloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_POINTCLOUD_INPUT_COMPONENT_HPP_