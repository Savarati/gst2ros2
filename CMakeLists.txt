cmake_minimum_required(VERSION 3.5)
project(mavgscam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra ")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wcomment")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers")

# Allow GNU extensions (-std=gnu++20)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

find_package(PkgConfig)
pkg_search_module(GST REQUIRED gstreamer-1.0)
pkg_search_module(GST_RTSP REQUIRED gstreamer-rtsp-server-1.0)
pkg_search_module(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
#find_package(pluginlib REQUIRED)
#find_package(rcpputils REQUIRED)
#find_package(message_filters REQUIRED)
find_package(mavlink REQUIRED)
find_package(mavros REQUIRED)
find_package(libmavconn REQUIRED)
find_package(console_bridge REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
#find_package(angles REQUIRED)
#find_package(eigen_stl_containers REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
#find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(std_msgs REQUIRED)
#find_package(std_srvs REQUIRED)
#find_package(trajectory_msgs REQUIRED)
#find_package(class_loader REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GST_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  include
)
include(CheckGeographicLibDatasets)

if(rclcpp_VERSION VERSION_LESS 9.0.0)
  add_definitions(
    -DUSE_OLD_DECLARE_PARAMETER
  )
endif()

add_library(mavgscam SHARED
  src/MavrosGstCamServer.cpp
  src/RosGstCamPublishStream.cpp
  src/RosGstCamConfig.cpp
  src/RosGstCamServer.cpp
)
ament_target_dependencies(mavgscam
  rclcpp
  rclcpp_components
  #rcpputils
  sensor_msgs
  #pluginlib
  mavros_msgs
  libmavconn
  mavros
  diagnostic_updater
  tf2_ros
  tf2_eigen
  #Eigen3
  #class_loader
  image_transport
  camera_calibration_parsers
  camera_info_manager
)
target_link_libraries(mavgscam
  ${ament_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${GST_RTSP_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  pthread
  yaml-cpp
)
rclcpp_components_register_nodes(mavgscam "mavros::cam::CAM" "campublishstream::CamPublishStream")

add_executable(mavgscam_node src/RosGstCamMain.cpp)
target_link_libraries(mavgscam_node mavgscam)

install(TARGETS mavgscam mavgscam_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}"
)

install(DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_include_directories(include)
ament_export_libraries(mavgscam)
#ament_export_dependencies(eigen3_cmake_module)
#ament_export_dependencies(Eigen3)
ament_package()
