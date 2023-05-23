/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@
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

/** \mainpage tracetools_benchmark: tracing tools and instrumentation for RobotPerf benchmarks
 *
 * `tracetools_benchmark` provides utilities to instrument ROS 2 selected packages
 *  that demonstrate hardware acceleration.
 *
 * It provides two main headers:
 *
 * - tracetools/tracetools.h
 *   - instrumentation functions
 * - tracetools/utils.hpp
 *   - utility functions
 */

#ifndef TRACETOOLS_BENCHMARK__TRACETOOLS_H_
#define TRACETOOLS_BENCHMARK__TRACETOOLS_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "tracetools_benchmark/config.h"
#include "tracetools_benchmark/visibility_control.hpp"

#ifndef TRACETOOLS_DISABLED
/// Call a tracepoint.
/**
 * This is the preferred method over calling the actual function directly.
 */
#  define TRACEPOINT(event_name, ...) \
  (ros_trace_ ## event_name)(__VA_ARGS__)
#  define DECLARE_TRACEPOINT(event_name, ...) \
  TRACETOOLS_PUBLIC void ros_trace_ ## event_name(__VA_ARGS__);
#else
#  define TRACEPOINT(event_name, ...) ((void) (0))
#  define DECLARE_TRACEPOINT(event_name, ...)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/// Get tracing compilation status.
/**
 * \return `true` if tracing is enabled, `false` otherwise
 */
TRACETOOLS_PUBLIC bool ros_trace_compile_status();

/// `robotperf_image_input_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::ImageInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_input_node rclcpp::node::Node subject to the callback
 * \param[in] image_input_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_input_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message
 */
DECLARE_TRACEPOINT(
  robotperf_image_input_cb_init,
  const void * image_input_node,
  const void * image_input_image_msg,
  const void * image_input_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg)

/// `robotperf_image_input_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::ImageOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_input_node rclcpp::node::Node subject to the callback
 * \param[in] image_input_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_input_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 */
DECLARE_TRACEPOINT(
  robotperf_image_input_cb_fini,
  const void * image_input_node,
  const void * image_input_image_msg,
  const void * image_input_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg)

/// `robotperf_image_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::ImageInputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_output_node rclcpp::node::Node subject to the callback
 * \param[in] image_output_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_output_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 */
DECLARE_TRACEPOINT(
  robotperf_image_output_cb_init,
  const void * image_output_node,
  const void * image_output_image_msg,
  const void * image_output_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg)


/// `robotperf_image_output_cb_fini`
/**
 * Tracepoint while finishing the callback of robotperf::perception::ImageOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] image_output_node rclcpp::node::Node subject to the callback
 * \param[in] image_output_image_msg image ROS message stored as sensor_msgs::msg::Image::ConstSharedPtr
 * \param[in] image_output_info_msg info ROS message as sensor_msgs::msg::CameraInfo::ConstSharedPtr
 * \param[in] image_input_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 * \param[in] image_input_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::Image's ROS message 
 */
DECLARE_TRACEPOINT(
  robotperf_image_output_cb_fini,
  const void * image_output_node,
  const void * image_output_image_msg,
  const void * image_output_info_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg)

/// `robotperf_pointcloud_output_cb_init`
/**
 * Tracepoint while initiating the callback of robotperf::perception::PointCloudOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] pointcloud_output_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_output_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] pointcloud_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] pointcloud_output_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_output_cb_init,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t pointcloud_output_header_nsec_arg,
  uint32_t pointcloud_output_header_sec_arg)

/// `robotperf_pointcloud_output_cb_init`
/**
 * Tracepoint while finishing the callback of robotperf::perception::PointCloudOutputComponent component
 *
 * Notes the `tracetools_benchmark` version automatically.
 *
 * \param[in] pointcloud_output_node rclcpp::node::Node subject to the callback
 * \param[in] pointcloud_output_pointcloud_msg pointcloud ROS message stored as sensor_msgs::msg::PointCloud2::ConstSharedPtr
 * \param[in] pointcloud_output_header_nsec_arg nanosec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 * \param[in] pointcloud_output_header_sec_arg sec field of the header (std_msgs/Header) of sensor_msgs::msg::PointCloud2's ROS message 
 */
DECLARE_TRACEPOINT(
  robotperf_pointcloud_output_cb_fini,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t pointcloud_output_header_nsec_arg,
  uint32_t pointcloud_output_header_sec_arg)

#ifdef __cplusplus
}
#endif

#endif  // TRACETOOLS_BENCHMARK__TRACETOOLS_H_
