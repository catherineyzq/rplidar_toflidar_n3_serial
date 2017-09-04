cmake_minimum_required(VERSION 2.8.3)
project(tof_ros)


set(TOF_SDK_PATH "./sdk")
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
  message_generation
)
add_message_files(
  FILES
  Tofpoint.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
 )
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS sdk/include
    DEPENDS rt pthread message_runtime
)


include_directories(
  ${TOF_SDK_PATH}/include
  ${catkin_INCLUDE_DIRS}
) 


add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)

add_executable(raw_node src/raw_node.cpp ${serial_SRCS})#必加的
add_dependencies(raw_node ${PROJECT_NAME})
add_dependencies(raw_node tof_ros_gencpp)
target_link_libraries(raw_node ${catkin_LIBRARIES})

install(TARGETS raw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
MESSAGE(STATUS "This is BINARY dir " ${tof_ros_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${tof_ros_SOURCE_DIR})

MESSAGE(STATUS "This is SOURCE dir " ${CATKIN_PACKAGE_LIB_DESTINATION})
MESSAGE(STATUS "This is SOURCE dir " ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install headers
install(FILES sdk/include/serial.h sdk/include/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)

