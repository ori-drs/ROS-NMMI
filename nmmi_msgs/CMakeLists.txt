cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(nmmi_msgs VERSION 2.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
    Info.msg
    ImuTable.msg
    ResourceData.msg
    State.msg
    ADCResourceData.msg
    ADC_State.msg
    EncoderResourceData.msg
    Encoder_State.msg
    inertialSensor.msg
    inertialSensorArray.msg
    quaternion.msg
    quaternionArray.msg
    temperature.msg
    temperatureArray.msg
    angles.msg
    anglesArray.msg
    adc.msg
    adcArray.msg
    encoder.msg
    encoderArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)