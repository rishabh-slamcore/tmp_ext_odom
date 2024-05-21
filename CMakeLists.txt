#[[=========================================================================== #
#
# Slamcore Confidential
# ---------------------
#
# Slamcore Limited
# All Rights Reserved.
# (C) Copyright 2018
#
# NOTICE:
#
# All information contained herein is, and remains the property of Slamcore
# Limited and its suppliers, if any. The intellectual and technical concepts
# contained herein are proprietary to Slamcore Limited and its suppliers and
# may be covered by patents in process, and are protected by trade secret or
# copyright law. Dissemination of this information or reproduction of this
# material is strictly forbidden unless prior written permission is obtained
# from Slamcore Limited.
#
# ============================================================================]]

# ============================================================================ #
# External dependencies
# ============================================================================ #
cmake_minimum_required(VERSION 3.10.2) # Ubuntu 18.04 LTS

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.16.0")
  cmake_policy(SET CMP0096 NEW) # Preserve leading zeros in version components
endif()

project(odom_sensor)

list(PREPEND CMAKE_PREFIX_PATH "/home/rishabh/work/position/build/install/lib/cmake/position")
list(PREPEND CMAKE_PREFIX_PATH "/home/rishabh/work/position/build/install/lib/cmake/slamcore_sdk")
list(PREPEND CMAKE_MODULE_PATH "/home/rishabh/work/position/build/install/lib/cmake/position;")
list(PREPEND CMAKE_MODULE_PATH "/home/rishabh/work/position/build/install/lib/cmake/slamcore_sdk")

message(${CMAKE_MODULE_PATH})

find_package(Threads REQUIRED)
find_package(slamcore_sdk REQUIRED)
find_package(position REQUIRED)
include(BuildOptions)
include(TopLevel)
configure_build_types()
# ============================================================================ #
# Build / simple_app
# ============================================================================ #
add_executable(sa Stream.cpp simple_app.cpp)
set_build_options(sa)

# Internal dependencies
target_link_libraries(sa PUBLIC slamcore_sdk)
target_include_directories(sa PUBLIC "/home/rishabh/work/position/build/install/include/slamcore")
# Threads
target_link_libraries(sa PRIVATE Threads::Threads)

get_target_property(esr sa INCLUDE_DIRECTORIES)
message(${esr})
# ============================================================================ #
# Strip
# ============================================================================ #
#strip_target(sa)
