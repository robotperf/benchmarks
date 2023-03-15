# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_a3_stereo_image_proc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED a3_stereo_image_proc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(a3_stereo_image_proc_FOUND FALSE)
  elseif(NOT a3_stereo_image_proc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(a3_stereo_image_proc_FOUND FALSE)
  endif()
  return()
endif()
set(_a3_stereo_image_proc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT a3_stereo_image_proc_FIND_QUIETLY)
  message(STATUS "Found a3_stereo_image_proc: 0.0.0 (${a3_stereo_image_proc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'a3_stereo_image_proc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${a3_stereo_image_proc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(a3_stereo_image_proc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${a3_stereo_image_proc_DIR}/${_extra}")
endforeach()
