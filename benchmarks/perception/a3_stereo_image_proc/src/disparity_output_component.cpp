#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a3_stereo_image_proc/disparity_output_component.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf
{

namespace perception
{

DisparityOutputComponent::DisparityOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("DisparityOutputComponent", options)
{

  //Create subscriber to Disparity image with callback function
  sub_disparity_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
    "/benchmark/disparity", rclcpp::QoS(10), std::bind(&DisparityOutputComponent::disparityCb, this, std::placeholders::_1));

}

void DisparityOutputComponent::disparityCb(
  const stereo_msgs::msg::DisparityImage::SharedPtr disparity_msg)
{
  TRACEPOINT(
    robotperf_image_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*disparity_msg)),
    nullptr,
    disparity_msg->header.stamp.nanosec
    );

  TRACEPOINT(
    robotperf_image_output_cb_fini,    
    static_cast<const void *>(this),
    static_cast<const void *>(&(*disparity_msg)),
    nullptr);

}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::DisparityOutputComponent)
