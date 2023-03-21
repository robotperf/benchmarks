#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a3_stereo_image_proc/images_input_component.hpp"

namespace robotperf
{

namespace perception
{

ImagesInputComponent::ImagesInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("ImagesInputComponent", options)
{
  // Create image publisher for left image and left image info
  pub_image_left_ = image_transport::create_camera_publisher(this, "image_input/left_input/left_image_raw");

  // Create image publisher for right image and right image info
  pub_image_right_ = image_transport::create_camera_publisher(this, "image_input/right_input/right_image_raw");

  // Create image sub for left camera. Include callback function
  sub_left_image_ = image_transport::create_camera_subscription(
    this, "left_camera/image_raw",
    std::bind(
      &ImagesInputComponent::leftImageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");

  // Create image sub for right camera. Include callback function
  sub_right_image_ = image_transport::create_camera_subscription(
    this, "right_camera/image_raw",
    std::bind(
      &ImagesInputComponent::rightImageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");

}

void ImagesInputComponent::leftImageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  // Record a tracepoint indicating the start of this callback function.
  // The tracepoint records the memory addresses of the `this`, `image_msg`,
  // and `info_msg` objects, which can be useful for performance analysis.
  TRACEPOINT(
    robotperf_image_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

  // If there are no subscribers to the image topic, do nothing and return
  if (pub_image_left_.getNumSubscribers() < 1) {
    return;
  }

  // Publish the image and camera info messages
  pub_image_left_.publish(*image_msg, *info_msg);

  // Record a tracepoint indicating the end of this callback function.
  TRACEPOINT(
    robotperf_image_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));
}


void ImagesInputComponent::rightImageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  // Record a tracepoint indicating the start of this callback function.
  // The tracepoint records the memory addresses of the `this`, `image_msg`,
  // and `info_msg` objects, which can be useful for performance analysis.
  TRACEPOINT(
    robotperf_image_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));

  // If there are no subscribers to the image topic, do nothing and return
  if (pub_image_right_.getNumSubscribers() < 1) {
    return;
  }

  // Publish the image and camera info messages
  pub_image_right_.publish(*image_msg, *info_msg);

  // Record a tracepoint indicating the end of this callback function.
  TRACEPOINT(
    robotperf_image_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)));
}


}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::ImagesInputComponent)