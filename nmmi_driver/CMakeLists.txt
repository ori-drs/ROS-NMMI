cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(nmmi_driver VERSION 2.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    nmmi_srvs
    qb_device_driver
    qb_device_utils
)

find_package(qb_device_driver REQUIRED)

include_directories(include
  api/src
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
    api/src
  CATKIN_DEPENDS
    roscpp
    nmmi_srvs
    qb_device_utils
    qb_device_driver
)

# Build Settings
add_executable(nmmi_communication_handler
  src/main.cpp
  src/nmmi_communication_handler.cpp
  api/src/cp_communications.cpp
  ${qb_device_driver_SOURCE_DIR}/api/src/qbmove_communications.cpp
  ${qb_device_driver_SOURCE_DIR}/src/qb_device_communication_handler.cpp
)

target_link_libraries(nmmi_communication_handler
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(nmmi_communication_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(nmmi_communication_handler
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      nmmi_communication_handler
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      nmmi_communication_handler
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)