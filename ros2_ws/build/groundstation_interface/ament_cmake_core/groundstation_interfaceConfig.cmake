# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_groundstation_interface_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED groundstation_interface_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(groundstation_interface_FOUND FALSE)
  elseif(NOT groundstation_interface_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(groundstation_interface_FOUND FALSE)
  endif()
  return()
endif()
set(_groundstation_interface_CONFIG_INCLUDED TRUE)

# output package information
if(NOT groundstation_interface_FIND_QUIETLY)
  message(STATUS "Found groundstation_interface: 0.0.0 (${groundstation_interface_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'groundstation_interface' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${groundstation_interface_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(groundstation_interface_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${groundstation_interface_DIR}/${_extra}")
endforeach()
