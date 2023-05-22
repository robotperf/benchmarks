/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@ Author: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
   @@@@@@@@@&@@@@@@@@@@
   @@@@@@@@@@@@@@@@@@@@

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a1_perception_2nodes/image_input_component.hpp"

namespace robotperf
{

namespace perception
{

ImageInputComponent::ImageInputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("ImageInputComponent", options)
{

  // Get the input_topic_name parameter from the parameter server with default value "input"
  std::string input_topic_name = this->declare_parameter<std::string>("input_topic_name", "input");

  // Create image pub
  pub_image_ = image_transport::create_camera_publisher(this, input_topic_name);
  // Create image sub
  sub_image_ = image_transport::create_camera_subscription(
    this, "image",
    std::bind(
      &ImageInputComponent::imageCb, this,
      std::placeholders::_1,
      std::placeholders::_2), "raw");
}

void ImageInputComponent::imageCb(
  sensor_msgs::msg::Image::ConstSharedPtr image_msg,
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
  TRACEPOINT(
    robotperf_image_input_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);

  if (pub_image_.getNumSubscribers() < 1) {
    return;
  }

  pub_image_.publish(*image_msg, *info_msg);

  TRACEPOINT(
    robotperf_image_input_cb_fini,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*image_msg)),
    static_cast<const void *>(&(*info_msg)),
    image_msg->header.stamp.nanosec,
    image_msg->header.stamp.sec);
}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::ImageInputComponent)