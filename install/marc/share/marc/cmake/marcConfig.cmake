# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_marc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED marc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(marc_FOUND FALSE)
  elseif(NOT marc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(marc_FOUND FALSE)
  endif()
  return()
endif()
set(_marc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT marc_FIND_QUIETLY)
  message(STATUS "Found marc: 1.0.0 (${marc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'marc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${marc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(marc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${marc_DIR}/${_extra}")
endforeach()
