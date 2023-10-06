cmake_minimum_required(VERSION 3.5)

project(ffmpeg_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# the image transport api changed between distros
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "foxy" OR
      $ENV{ROS_DISTRO} STREQUAL "galactic")
    add_definitions(-DUSE_OLD_IMAGE_TRANSPORT_API)
  endif()
else()
  message(ERROR "ROS_DISTRO environment variable is not set!")
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(python_cmake_module REQUIRED)

find_package(OpenCV REQUIRED imgproc)
find_package(pybind11 REQUIRED)
_ament_cmake_python_register_environment_hook()

# Cannot use pybind11_add_module, since it uses the keyword version of target_link_libraries (PUBLIC, PRIVATE, ...),
# so we create it ourselves.
add_library(ffmpeg_bindings SHARED src/ffmpeg_decoder_pybind.cpp)

set_target_properties(ffmpeg_bindings PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")


target_include_directories(ffmpeg_bindings PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


target_include_directories(ffmpeg_bindings PUBLIC ${rclcpp_INCLUDE_DIRS})

ament_target_dependencies(
  ffmpeg_bindings
  rclcpp
  cv_bridge
  sensor_msgs
  camera_info_manager
  OpenCV
  camera_calibration_parsers
  ffmpeg_image_transport_msgs
)
target_link_libraries(ffmpeg_bindings pybind11::module)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ffmpeg_bindings
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ffmpeg_bindings
  DESTINATION "${PYTHON_INSTALL_DIR}"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_interfaces(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  ffmpeg_bindings
)

ament_package()