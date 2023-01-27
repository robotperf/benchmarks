#ifndef ROBOTPERF_IMAGE_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_IMAGE_OUTPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>

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

class ImageOutputComponent
  : public rclcpp::Node
{
public:
  explicit ImageOutputComponent(const rclcpp::NodeOptions &);

protected:
  image_transport::CameraSubscriber sub_image_;

  void imageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPER_AIAMOUTPUTPUT_COMPONENT_HPP_