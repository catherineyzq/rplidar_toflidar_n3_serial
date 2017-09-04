cmake_minimum_required(VERSION 2.8.3)
project(serial_collect)

set(SERIAL_COLLECT_SDK_PATH "./sdk")
set(serial_SRCS
    sdk/src/serial.cc
    sdk/include/serial.h
    sdk/include/v8stdint.h
    sdk/src/impl/list_ports/list_ports_linux.cc
    sdk/src/impl/unix.cc
)
# Find catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  std_msgs
  rplidar_ros
  tof_ros
  dji_sdk
)
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    DEPENDS rt pthread
)
include_directories(
  ${SERIAL_COLLECT_SDK_PATH}/include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)
add_executable(get3node src/get3node.cpp ${serial_SRCS})#必加的
add_dependencies(get3node ${PROJECT_NAME})
add_dependencies(get3node tof_ros_gencpp)
target_link_libraries(get3node ${catkin_LIBRARIES})

install(TARGETS get3node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
