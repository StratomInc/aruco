cmake_minimum_required(VERSION 3.5)
project(aruco)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#Enable C++ 11
add_compile_options(-std=c++11)

#Packages
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(msg_files
  "msg/Marker.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

set(node_plugins "")

add_library(aruco_detector
  src/ros/Aruco.cpp
  src/ros/aruco_composition.cpp
  src/ros/aruco_tf_producer.cpp)
ament_target_dependencies(aruco_detector cv_bridge image_transport OpenCV rclcpp rclcpp_components std_msgs)

add_library(aruco_component SHARED
  src/ros/Aruco.cpp)
target_compile_definitions(aruco_component
  PRIVATE "COMPOSITION_BUILDIG_DLL")
ament_target_dependencies(aruco_component
  cv_bridge
  image_transport
  OpenCV
  rclcpp
  rclcpp_components
  std_msgs)
rclcpp_components_register_nodes(aruco_component "aruco::Aruco")
set(node_plugins "${node_plugin}aruco::Aruco;$<TARGET_FILE:Aruco>\n")

add_library(aruco_tf_producer_component SHARED
  src/ros/aruco_tf_producer.cpp)
target_compile_definitions(aruco_tf_producer_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(aruco_tf_producer_component
  orocos_kdl
  rclcpp
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros)
rclcpp_components_register_nodes(aruco_tf_producer_component "aruco::ArucoTFProducer")
set(node_plugins "${node_plugins}aruco::ArucoTFProducer;$<TARGET_FILE:aruco_tf_producer>\n")

add_executable(aruco_composition
  src/ros/aruco_composition.cpp)
target_link_libraries(aruco_composition
  aruco_component
  aruco_tf_producer_component)
ament_target_dependencies(aruco_composition rclcpp)

install(TARGETS
  aruco_component
  aruco_tf_producer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  aruco_composition
  DESTINATION lib/${PROJECT_NAME}/)

rosidl_target_interfaces(aruco_detector ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(aruco_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(aruco_composition ${PROJECT_NAME} "rosidl_typesupport_cpp")


#############
## TESTING ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  find_package(ament_cmake_gmock)
endif()

ament_package()
