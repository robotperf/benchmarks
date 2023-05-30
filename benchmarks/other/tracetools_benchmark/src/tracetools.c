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

#include "tracetools_benchmark/tracetools.h"

#ifndef TRACETOOLS_DISABLED

#ifdef TRACETOOLS_LTTNG_ENABLED
# include "tracetools_benchmark/tp_call.h"
# define CONDITIONAL_TP(...) \
  tracepoint(TRACEPOINT_PROVIDER, __VA_ARGS__)
#else
# define CONDITIONAL_TP(...)
#endif

bool ros_trace_compile_status()
{
#ifdef TRACETOOLS_LTTNG_ENABLED
  return true;
#else
  return false;
#endif
}

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#else
# pragma warning(push)
# pragma warning(disable: 4100)
#endif

// image_input
void TRACEPOINT(
  robotperf_image_input_cb_init,
  const void * image_input_node_arg,
  const void * image_input_image_msg_arg,
  const void * image_input_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size_arg,
  size_t image_input_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_input_cb_init,
    image_input_node_arg,
    image_input_image_msg_arg,
    image_input_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_input_image_msg_size_arg,
    image_input_info_msg_size_arg);
}
void TRACEPOINT(
  robotperf_image_input_cb_fini,
  const void * image_input_node_arg,
  const void * image_input_image_msg_arg,
  const void * image_input_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_input_image_msg_size_arg,
  size_t image_input_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_input_cb_fini,
    image_input_node_arg,
    image_input_image_msg_arg,
    image_input_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_input_image_msg_size_arg,
    image_input_info_msg_size_arg);
}

// image_output
void TRACEPOINT(
  robotperf_image_output_cb_init,
  const void * image_output_node_arg,
  const void * image_output_image_msg_arg,
  const void * image_output_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size_arg,
  size_t image_output_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_output_cb_init,
    image_output_node_arg,
    image_output_image_msg_arg,
    image_output_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_output_image_msg_size_arg,
    image_output_info_msg_size_arg);
}
void TRACEPOINT(
  robotperf_image_output_cb_fini,
  const void * image_output_node_arg,
  const void * image_output_image_msg_arg,
  const void * image_output_info_msg_arg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t image_output_image_msg_size_arg,
  size_t image_output_info_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_image_output_cb_fini,
    image_output_node_arg,
    image_output_image_msg_arg,
    image_output_info_msg_arg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    image_output_image_msg_size_arg,
    image_output_info_msg_size_arg);
}

void TRACEPOINT(
  robotperf_pointcloud_output_cb_init,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_output_cb_init,
    pointcloud_output_node,
    pointcloud_output_pointcloud_msg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    pointcloud_output_msg_size_arg);
}

void TRACEPOINT(
  robotperf_pointcloud_output_cb_fini,
  const void * pointcloud_output_node,
  const void * pointcloud_output_pointcloud_msg,
  uint32_t image_input_header_nsec_arg,
  uint32_t image_input_header_sec_arg,
  size_t pointcloud_output_msg_size_arg)
{
  CONDITIONAL_TP(
    robotperf_pointcloud_output_cb_fini,
    pointcloud_output_node,
    pointcloud_output_pointcloud_msg,
    image_input_header_nsec_arg,
    image_input_header_sec_arg,
    pointcloud_output_msg_size_arg);
}

#ifndef _WIN32
# pragma GCC diagnostic pop
#else
# pragma warning(pop)
#endif

#endif  // TRACETOOLS_DISABLED
