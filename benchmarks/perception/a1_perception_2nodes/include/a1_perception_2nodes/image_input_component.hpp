#ifndef ROBOTPERF_IMAGE_INPUT_COMPONENT_HPP_
#define ROBOTPERF_IMAGE_INPUT_COMPONENT_HPP_

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

class ImageInputComponent
  : public rclcpp::Node
{
public:
  explicit ImageInputComponent(const rclcpp::NodeOptions &);

protected:
  image_transport::CameraPublisher pub_image_;
  image_transport::CameraSubscriber sub_image_;

  void imageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_IMAGE_INPUT_COMPONENT_HPP_