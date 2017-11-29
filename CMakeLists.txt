cmake_minimum_required(VERSION 2.8.3)
project(ros_libvlc)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules
)

find_package(libvlc REQUIRED)
set(LIBVLC_LIBRARIES ${LIBVLC_LIBRARY})

# FIXME: use target_compile_features instead
set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
# ros_libvlc cannot be used as a library
)

# TODO: Check names of system library include directories (libvlc)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBVLC_INCLUDE_DIRS}
)

add_executable(ros_libvlc src/main.cpp src/RosLibVlc.cpp)

target_link_libraries(ros_libvlc
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBVLC_LIBRARIES}
)

install(TARGETS ros_libvlc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)