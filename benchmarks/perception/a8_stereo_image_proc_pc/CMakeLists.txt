cmake_minimum_required(VERSION 3.5)
project(a8_stereo_image_proc_pc)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(tracetools_benchmark REQUIRED)

find_package(stereo_image_proc REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Nodes built as "components"
ament_auto_add_library(disparity_input_component SHARED src/disparity_input_component.cpp)
ament_target_dependencies(disparity_input_component tracetools_benchmark image_transport)
target_compile_definitions(disparity_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(disparity_input_component "robotperf::perception::DisparityInputComponent")
set(node_plugins "${node_plugins}robotperf::perception::DisparityInputComponent;$<TARGET_FILE:disparity_input_component>\n")

ament_auto_add_library(pc_output_component SHARED src/pc_output_component.cpp)
ament_target_dependencies(pc_output_component tracetools_benchmark image_transport)
target_compile_definitions(pc_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(pc_output_component "robotperf::perception::PcOutputComponent")
set(node_plugins "${node_plugins}robotperf::perception::PcOutputComponent;$<TARGET_FILE:pc_output_component>\n")

# install(
#   TARGETS ${install_targets}
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files.
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package(INSTALL_TO_SHARE launch)
