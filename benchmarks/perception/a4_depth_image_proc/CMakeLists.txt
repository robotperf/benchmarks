cmake_minimum_required(VERSION 3.5)
project(a4_depth_image_proc)

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

find_package(depth_image_proc REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# LTTng flame chart and graphs
# see https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html#Flame_Chart_View
set(CMAKE_CXX_FLAGS "-g -O2 -finstrument-functions")

# Nodes built as "components"
ament_auto_add_library(point_cloud_output_component SHARED src/point_cloud_output_component.cpp)
ament_target_dependencies(point_cloud_output_component tracetools_benchmark image_transport)
target_compile_definitions(point_cloud_output_component PRIVATE "COMPOSITION_BUILDING_DLL")
rclcpp_components_register_nodes(point_cloud_output_component "robotperf::perception::PointCloudOutputComponent")
set(node_plugins "${node_plugins}robotperf::perception::PointCloudOutputComponent;$<TARGET_FILE:point_cloud_output_component>\n")


# install(
#   TARGETS ${install_targets}
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files.
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# # Install description directory.
# install(
#   DIRECTORY description
#   DESTINATION share/${PROJECT_NAME}
# )

ament_auto_package(INSTALL_TO_SHARE launch)
# ament_package()

