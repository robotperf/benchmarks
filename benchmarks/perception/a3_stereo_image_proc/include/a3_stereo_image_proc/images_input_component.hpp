#ifndef ROBOTPERF_IMAGES_INPUT_COMPONENT_HPP_
#define ROBOTPERF_IMAGES_INPUT_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_resource.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "tracetools_benchmark/tracetools.h"

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

class ImagesInputComponent
  : public rclcpp::Node
{
public:
  explicit ImagesInputComponent(const rclcpp::NodeOptions &);

private:
  //Node to publish left images and info
  image_transport::CameraPublisher pub_image_left_;
  //Node to publish right images amd info
  image_transport::CameraPublisher pub_image_right_;

  //Subscriber to left image topic
  image_transport::CameraSubscriber sub_left_image_;
  //Subscriber to right image topic
  image_transport::CameraSubscriber sub_right_image_;

  // Left Image Callback function called when new image message and camera info message are received from the image transport.
  // The function processes these messages and publishes the image message with its camera info to a image_transport::CameraPublisher.
  void leftImageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

  // Right Image Callback function called when new image message and camera info message are received from the image transport.
  // The function processes these messages and publishes the image message with its camera info to a image_transport::CameraPublisher.
  void rightImageCb(
    sensor_msgs::msg::Image::ConstSharedPtr image_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);

};

}  // namespace perception

}  // namespace robotperf

#endif  // ROBOTPERF_IMAGES_INPUT_COMPONENT_HPP_


