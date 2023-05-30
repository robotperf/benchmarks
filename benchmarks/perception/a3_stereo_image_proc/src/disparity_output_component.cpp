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
#include <stereo_msgs/msg/disparity_image.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a3_stereo_image_proc/disparity_output_component.hpp"
#include <rclcpp/serialization.hpp>

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

size_t DisparityOutputComponent::get_msg_size(stereo_msgs::msg::DisparityImage::ConstSharedPtr disparity_msg){
  //Serialize the Image and CameraInfo messages
  rclcpp::SerializedMessage serialized_data_disp_img;
  rclcpp::Serialization<stereo_msgs::msg::DisparityImage> disp_image_serialization;
  const void* disp_image_ptr = reinterpret_cast<const void*>(disparity_msg.get());
  disp_image_serialization.serialize_message(disp_image_ptr, &serialized_data_disp_img);
  size_t disparity_msg_size = serialized_data_disp_img.size();
  return disparity_msg_size;
}

void DisparityOutputComponent::disparityCb(
  const stereo_msgs::msg::DisparityImage::SharedPtr disparity_msg)
{
  TRACEPOINT(
    robotperf_image_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*disparity_msg)),
    nullptr,
    disparity_msg->header.stamp.nanosec,
    disparity_msg->header.stamp.sec,
    get_msg_size(disparity_msg),
    size_t(0));

  TRACEPOINT(
    robotperf_image_output_cb_fini,    
    static_cast<const void *>(this),
    static_cast<const void *>(&(*disparity_msg)),
    nullptr,
    disparity_msg->header.stamp.nanosec,
    disparity_msg->header.stamp.sec,
    get_msg_size(disparity_msg),
    size_t(0));

}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::DisparityOutputComponent)
