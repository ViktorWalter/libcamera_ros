cmake_minimum_required(VERSION 3.5)
project(libcamera_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  nodelet
  sensor_msgs
  std_msgs
  )

set(LIBRARIES
  LibCameraRos
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

catkin_package(
  LIBRARIES ${LIBRARIES}
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

find_package(PkgConfig REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${libcamera_INCLUDE_DIRS}
  )


# library with common utility functions for type conversions
add_library(LibCameraRos 
  src/LibcameraRos.cpp
  src/utils/clamp.cpp
  src/utils/format_mapping.cpp
  src/utils/pretty_print.cpp
  src/utils/types.cpp
  src/utils/type_extent.cpp
)


target_link_libraries(LibCameraRos
  ${catkin_LIBRARIES}
  ${libcamera_LINK_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
