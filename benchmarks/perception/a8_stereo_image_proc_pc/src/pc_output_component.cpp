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
#include <sensor_msgs/msg/point_cloud2.hpp>


#include "tracetools_benchmark/tracetools.h"
#include "a8_stereo_image_proc_pc/pc_output_component.hpp"
#include <rclcpp/serialization.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf
{

namespace perception
{

PcOutputComponent::PcOutputComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("PcOutputComponent", options)
{
  // Get the output_topic_name parameter from the parameter server with default value "input"
  std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name", "/benchmark/pc");

  //Create subscriber to Point cloud with callback function
  sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    output_topic_name, rclcpp::QoS(10), std::bind(&PcOutputComponent::pcCb, this, std::placeholders::_1));

}

size_t PcOutputComponent::get_msg_size(sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg){
  //Serialize the Image and CameraInfo messages
  rclcpp::SerializedMessage serialized_data_pc_img;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_image_serialization;
  const void* pc_image_ptr = reinterpret_cast<const void*>(pc_msg.get());
  pc_image_serialization.serialize_message(pc_image_ptr, &serialized_data_pc_img);
  size_t pc_msg_size = serialized_data_pc_img.size();
  return pc_msg_size;
}

void PcOutputComponent::pcCb(
  const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg)
{
  TRACEPOINT(
    robotperf_image_output_cb_init,
    static_cast<const void *>(this),
    static_cast<const void *>(&(*pc_msg)),
    nullptr,
    pc_msg->header.stamp.nanosec,
    pc_msg->header.stamp.sec,
    get_msg_size(pc_msg),
    size_t(0));

  TRACEPOINT(
    robotperf_image_output_cb_fini,    
    static_cast<const void *>(this),
    static_cast<const void *>(&(*pc_msg)),
    nullptr,
    pc_msg->header.stamp.nanosec,
    pc_msg->header.stamp.sec,
    get_msg_size(pc_msg),
    size_t(0));

}

}  // namespace perception

}  // namespace robotperf

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::PcOutputComponent)
