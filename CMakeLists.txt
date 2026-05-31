cmake_minimum_required(VERSION 3.10)
project(le_vins)

## Build Options

set(CMAKE_CXX_STANDARD 17)
if (CMAKE_BUILD_TYPE STREQUAL Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
endif()

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# LE-VINS Library
add_subdirectory(le_vins)

# abseil
add_subdirectory(thirdparty/abseil-cpp-20230125.0)

## Executable

# ROS interface
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(tf2_ros REQUIRED)

# LE-VINS ROS
add_executable(${PROJECT_NAME}_ros
    ROS/main.cc
    ROS/fusion.cc
    ROS/visual/visual_drawer_rviz.cc
    ROS/lidar/lidar_converter.cc)

target_include_directories(${PROJECT_NAME}_ros PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/le_vins)
ament_target_dependencies(${PROJECT_NAME}_ros
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    pcl_conversions
    rosbag2_cpp
    rosbag2_storage
    tf2_ros)

ament_target_dependencies(${PROJECT_NAME}_ros livox_ros_driver2)

target_link_libraries(${PROJECT_NAME}_ros
    le_vins)

# PCL
find_package(PCL REQUIRED)
target_include_directories(${PROJECT_NAME}_ros PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}_ros PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(${PROJECT_NAME}_ros ${PCL_LIBRARIES})

# abseil
target_link_libraries(${PROJECT_NAME}_ros
    absl::strings
    absl::time)

install(TARGETS ${PROJECT_NAME}_ros
    DESTINATION lib/${PROJECT_NAME})
install(TARGETS
    le_vins
    fileio
    preintegration
    visual
    lidar
    factors
    common
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
