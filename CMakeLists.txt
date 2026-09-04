cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(crawler_mujoco)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(truth_cloud_publisher src/truth_cloud_publisher.cpp)
target_include_directories(truth_cloud_publisher PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(truth_cloud_publisher ${PCL_LIBRARIES})
ament_target_dependencies(truth_cloud_publisher
  pcl_conversions
  rclcpp
  sensor_msgs
)

add_executable(voxel_overhang_removal src/voxel_overhang_removal.cpp)
target_include_directories(voxel_overhang_removal PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(voxel_overhang_removal ${PCL_LIBRARIES})
ament_target_dependencies(voxel_overhang_removal
  pcl_conversions
  rclcpp
  sensor_msgs
)

install(TARGETS truth_cloud_publisher voxel_overhang_removal
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/crawler_mujoco DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz assets docs picture
  DESTINATION share/${PROJECT_NAME}
)
install(FILES requirements.txt DESTINATION share/${PROJECT_NAME})
install(FILES README.md CHANGELOG.rst LICENSE THIRD_PARTY_NOTICES.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
