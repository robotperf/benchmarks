cmake_minimum_required(VERSION 3.5)
project(a1_perception_2nodes)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED)

# find_package(ament_vitis QUIET) # enable Vitis capabilities, if available
# find_package(ament_cuda QUIET)  # enable CUDA capabilities, if available
find_package(tracetools_benchmark REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Nodes built as "components"
ament_auto_add_library(image_input_component SHARED src/image_input_component.cpp)
ament_target_dependencies(image_input_component tracetools_benchmark image_transport)
target_compile_definitions(image_input_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(image_input_component "robotperf::perception::ImageInputComponent")
set(node_plugins "${node_plugins}robotperf::perception::ImageInputComponent;$<TARGET_FILE:image_input_component>\n")

ament_auto_add_library(image_output_component SHARED src/image_output_component.cpp)
ament_target_dependencies(image_output_component tracetools_benchmark image_transport)
target_compile_definitions(image_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(image_output_component "robotperf::perception::ImageOutputComponent")
set(node_plugins "${node_plugins}robotperf::perception::ImageOutputComponent;$<TARGET_FILE:image_output_component>\n")

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

# ament_package()
