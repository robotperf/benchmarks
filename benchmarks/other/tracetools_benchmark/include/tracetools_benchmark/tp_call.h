/*
   @@@@@@@@@@@@@@@@@@@@
   @@@@@@@@@&@@@&&@@@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@
   @@@@@ @@  @@    @@@@ Copyright (c) 2023, Acceleration Robotics®
   @@@@@ @@  @@    @@@@ Author: Víctor Mayoral Vilches <victor@accelerationrobotics.com>
   @@@@@ @@  @@    @@@@ AUthor: Alejandra Martínez Fariña <alex@accelerationrobotics.com>
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

// Provide fake header guard for cpplint
#undef TRACETOOLS_BENCHMARK__TP_CALL_H_
#ifndef TRACETOOLS_BENCHMARK__TP_CALL_H_
#define TRACETOOLS_BENCHMARK__TP_CALL_H_

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER robotperf_benchmarks

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "tracetools_benchmark/tp_call.h"

#if !defined(_TRACETOOLS_BENCHMARK__TP_CALL_H_) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRACETOOLS_BENCHMARK__TP_CALL_H_

#include <lttng/tracepoint.h>
#include <stdint.h>
#include <stdbool.h>

// robotperf image_input init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,              // tracepoint provider name
  robotperf_image_input_cb_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, image_input_node_arg,
    const void *, image_input_image_msg_arg,
    const void *, image_input_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_input_image_msg_size_arg,
    size_t, image_input_info_msg_size_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, image_input_node, image_input_node_arg)
    ctf_integer_hex(const void *, image_input_image_msg, image_input_image_msg_arg)
    ctf_integer_hex(const void *, image_input_info_msg, image_input_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_input_image_msg_size, image_input_image_msg_size_arg)
    ctf_integer(size_t, image_input_info_msg_size, image_input_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)
// robotperf image_input end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_image_input_cb_fini,
  TP_ARGS(
    const void *, image_input_node_arg,
    const void *, image_input_image_msg_arg,
    const void *, image_input_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_input_image_msg_size_arg,
    size_t, image_input_info_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, image_input_node, image_input_node_arg)
    ctf_integer_hex(const void *, image_input_image_msg, image_input_image_msg_arg)
    ctf_integer_hex(const void *, image_input_info_msg, image_input_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_input_image_msg_size, image_input_image_msg_size_arg)
    ctf_integer(size_t, image_input_info_msg_size, image_input_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

// robotperf image_output init callback
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,              // tracepoint provider name
  robotperf_image_output_cb_init,    // tracepoint name
  TP_ARGS(
    // input arguments, see https://lttng.org/docs/v2.12/#doc-tpp-def-input-args
    const void *, image_output_node_arg,
    const void *, image_output_image_msg_arg,
    const void *, image_output_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_output_image_msg_size_arg,
    size_t, image_output_info_msg_size_arg),
  TP_FIELDS(
    // output event fields, see https://lttng.org/man/3/lttng-ust/v2.12/#doc-ctf-macros
    ctf_integer_hex(const void *, image_output_node, image_output_node_arg)
    ctf_integer_hex(const void *, image_output_image_msg, image_output_image_msg_arg)
    ctf_integer_hex(const void *, image_output_info_msg, image_output_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_output_image_msg_size, image_output_image_msg_size_arg)
    ctf_integer(size_t, image_output_info_msg_size, image_output_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)
// robotperf image_output end of callback (after publication)
TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_image_output_cb_fini,
  TP_ARGS(
    const void *, image_output_node_arg,
    const void *, image_output_image_msg_arg,
    const void *, image_output_info_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, image_output_image_msg_size_arg,
    size_t, image_output_info_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, image_output_node, image_output_node_arg)
    ctf_integer_hex(const void *, image_output_image_msg, image_output_image_msg_arg)
    ctf_integer_hex(const void *, image_output_info_msg, image_output_info_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, image_output_image_msg_size, image_output_image_msg_size_arg)
    ctf_integer(size_t, image_output_info_msg_size, image_output_info_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_output_cb_init,
  TP_ARGS(
    const void *, pointcloud_output_node_arg,
    const void *, pointcloud_output_pointcloud_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, pointcloud_output_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_output_node, pointcloud_output_node_arg)
    ctf_integer_hex(const void *, pointcloud_output_pointcloud_msg, pointcloud_output_pointcloud_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_output_msg_size, pointcloud_output_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

TRACEPOINT_EVENT(
  TRACEPOINT_PROVIDER,
  robotperf_pointcloud_output_cb_fini,
  TP_ARGS(
    const void *, pointcloud_output_node_arg,
    const void *, pointcloud_output_pointcloud_msg_arg,
    uint32_t, image_input_header_nsec_arg,
    uint32_t, image_input_header_sec_arg,
    size_t, pointcloud_output_msg_size_arg),
  TP_FIELDS(
    ctf_integer_hex(const void *, pointcloud_output_node, pointcloud_output_node_arg)
    ctf_integer_hex(const void *, pointcloud_output_pointcloud_msg, pointcloud_output_pointcloud_msg_arg)
    ctf_integer(uint32_t, image_input_header_nsec, image_input_header_nsec_arg)
    ctf_integer(uint32_t, image_input_header_sec, image_input_header_sec_arg)
    ctf_integer(size_t, pointcloud_output_msg_size, pointcloud_output_msg_size_arg)
    ctf_string(version, tracetools_benchmark_VERSION)
  )
)

#endif  // _TRACETOOLS_BENCHMARK__TP_CALL_H_

#include <lttng/tracepoint-event.h>

#endif  // TRACETOOLS_BENCHMARK__TP_CALL_H_
