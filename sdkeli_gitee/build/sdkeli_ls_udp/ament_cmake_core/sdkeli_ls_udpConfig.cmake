# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sdkeli_ls_udp_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sdkeli_ls_udp_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sdkeli_ls_udp_FOUND FALSE)
  elseif(NOT sdkeli_ls_udp_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sdkeli_ls_udp_FOUND FALSE)
  endif()
  return()
endif()
set(_sdkeli_ls_udp_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sdkeli_ls_udp_FIND_QUIETLY)
  message(STATUS "Found sdkeli_ls_udp: 0.0.1 (${sdkeli_ls_udp_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sdkeli_ls_udp' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT sdkeli_ls_udp_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sdkeli_ls_udp_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${sdkeli_ls_udp_DIR}/${_extra}")
endforeach()
