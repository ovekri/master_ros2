# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_plane_fitter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED plane_fitter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(plane_fitter_FOUND FALSE)
  elseif(NOT plane_fitter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(plane_fitter_FOUND FALSE)
  endif()
  return()
endif()
set(_plane_fitter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT plane_fitter_FIND_QUIETLY)
  message(STATUS "Found plane_fitter: 0.0.0 (${plane_fitter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'plane_fitter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${plane_fitter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(plane_fitter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${plane_fitter_DIR}/${_extra}")
endforeach()
