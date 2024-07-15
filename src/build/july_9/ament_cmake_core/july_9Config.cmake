# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_july_9_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED july_9_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(july_9_FOUND FALSE)
  elseif(NOT july_9_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(july_9_FOUND FALSE)
  endif()
  return()
endif()
set(_july_9_CONFIG_INCLUDED TRUE)

# output package information
if(NOT july_9_FIND_QUIETLY)
  message(STATUS "Found july_9: 0.0.0 (${july_9_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'july_9' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${july_9_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(july_9_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${july_9_DIR}/${_extra}")
endforeach()
