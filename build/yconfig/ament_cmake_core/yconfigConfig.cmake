# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_yconfig_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED yconfig_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(yconfig_FOUND FALSE)
  elseif(NOT yconfig_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(yconfig_FOUND FALSE)
  endif()
  return()
endif()
set(_yconfig_CONFIG_INCLUDED TRUE)

# output package information
if(NOT yconfig_FIND_QUIETLY)
  message(STATUS "Found yconfig: 0.3.0 (${yconfig_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'yconfig' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${yconfig_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(yconfig_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${yconfig_DIR}/${_extra}")
endforeach()
