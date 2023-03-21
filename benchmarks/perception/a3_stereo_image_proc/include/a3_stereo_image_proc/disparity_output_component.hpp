#ifndef ROBOTPERF_DISPARITY_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_DISPARITY_OUTPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <thread>

namespace robotperf
{

namespace perception
{

class DisparityOutputComponent
  : public rclcpp::Node
{
public:
  explicit DisparityOutputComponent(const rclcpp::NodeOptions &);

protected:
  // Create subscriber to Disparity image with callback function
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_disparity_;


  void disparityCb(const stereo_msgs::msg::DisparityImage::SharedPtr disparity_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPER_AIAMOUTPUTPUT_COMPONENT_HPP_






