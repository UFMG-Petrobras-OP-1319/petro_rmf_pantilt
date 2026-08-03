cmake_minimum_required(VERSION 3.8)
project(hk_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable compiler optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(CURL REQUIRED) # the PTZ node talks ISAPI over HTTP
find_package(hk_camera_interfaces REQUIRED) # custom messages and services

# Locate the HIKROBOT MVS SDK (downloaded separately from the vendor site, usually /opt/MVS).
# When it is missing, skip the nodes that need a camera and build only the rest, so that
# the whole package does not fail to compile.
set(MVS_ROOT "/opt/MVS" CACHE PATH "HIKROBOT MVS SDK install path")
find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS ${MVS_ROOT}/include
)
find_library(MVS_CAMERA_CONTROL_LIBRARY
  NAMES MvCameraControl
  PATHS ${MVS_ROOT}/lib/64 ${MVS_ROOT}/lib/aarch64
)

if(MVS_INCLUDE_DIR AND MVS_CAMERA_CONTROL_LIBRARY)
  set(MVS_FOUND TRUE)
  get_filename_component(MVS_LIBRARY_DIR ${MVS_CAMERA_CONTROL_LIBRARY} DIRECTORY)
  message(STATUS "Found MVS SDK: ${MVS_INCLUDE_DIR}, ${MVS_LIBRARY_DIR}")
else()
  set(MVS_FOUND FALSE)
  message(WARNING
    "HIKROBOT MVS SDK not found (no MvCameraControl.h or libMvCameraControl under ${MVS_ROOT}).\n"
    "  All nodes that need an industrial camera will be skipped; the RTSP, PTZ and client\n"
    "  nodes are still built.\n"
    "  Download and install the MVS SDK from https://www.hikrobotics.com, then rebuild.")
endif()

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  include
)

# MVS libraries shared by all industrial camera nodes
set(MVS_LIBRARIES
  MvCameraControl
  MVGigEVisionSDK
  MVRender
  MvUsb3vTL
  MediaProcess
  X11
)

set(INSTALL_TARGETS "")

# Nodes that need the industrial camera: source file -> target name
if(MVS_FOUND)
  set(MVS_NODES
    hk_camera
    hk_camera_compressed_pub
    hk_camera_compressed_sub
    hk_camera_compressed_pub_test
    hk_camera_compressed_sub_test
    hk_camera_compressed_pub_test_reliable
    hk_camera_compressed_sub_test_reliable
  )

  foreach(node ${MVS_NODES})
    add_executable(${node} src/${node}.cpp)
    ament_target_dependencies(${node}
      rclcpp
      std_msgs
      sensor_msgs
      cv_bridge
      image_transport
    )
    target_include_directories(${node} PRIVATE ${MVS_INCLUDE_DIR})
    target_link_directories(${node} PRIVATE ${MVS_LIBRARY_DIR})
    target_link_libraries(${node}
      ${OpenCV_LIBRARIES}
      ${MVS_LIBRARIES}
    )
    list(APPEND INSTALL_TARGETS ${node})
  endforeach()

  # Service server (needs camera access)
  add_executable(hk_camera_server src/hk_camera_server.cpp)
  ament_target_dependencies(hk_camera_server
    rclcpp
    std_msgs
    sensor_msgs
    hk_camera_interfaces
  )
  target_include_directories(hk_camera_server PRIVATE ${MVS_INCLUDE_DIR})
  target_link_directories(hk_camera_server PRIVATE ${MVS_LIBRARY_DIR})
  target_link_libraries(hk_camera_server
    ${OpenCV_LIBRARIES}
    ${MVS_LIBRARIES}
  )
  list(APPEND INSTALL_TARGETS hk_camera_server)
endif()

# Network camera (RTSP) node: streams via OpenCV/FFMPEG, no MVS SDK required
add_executable(hk_camera_rtsp_pub src/hk_camera_rtsp_pub.cpp)
ament_target_dependencies(hk_camera_rtsp_pub
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  hk_camera_interfaces # the take_photo service
)
target_link_libraries(hk_camera_rtsp_pub
  ${OpenCV_LIBRARIES}
)
list(APPEND INSTALL_TARGETS hk_camera_rtsp_pub)

# Network camera PTZ node: uses the ISAPI HTTP interface, no MVS SDK required
add_executable(hk_camera_ptz src/hk_camera_ptz.cpp)
ament_target_dependencies(hk_camera_ptz
  rclcpp
  std_msgs
  geometry_msgs
)
target_link_libraries(hk_camera_ptz
  CURL::libcurl
)
list(APPEND INSTALL_TARGETS hk_camera_ptz)

# The client only calls the service and never touches the camera, so no MVS SDK required
add_executable(hk_camera_client src/hk_camera_client.cpp)
ament_target_dependencies(hk_camera_client
  rclcpp
  std_msgs
  hk_camera_interfaces
)
target_link_libraries(hk_camera_client
  ${OpenCV_LIBRARIES}
)
list(APPEND INSTALL_TARGETS hk_camera_client)

install(TARGETS
  ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, parameter files and the rviz config
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
