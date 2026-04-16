cmake_minimum_required(VERSION 3.8)
project(apriltag_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(amr_ex_action REQUIRED)
find_package(sim_world_msgs REQUIRED)

include_directories(include)

add_library(apriltag_sim_node_lib
  src/apriltag_sim_node.cpp
)
ament_target_dependencies(apriltag_sim_node_lib
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs amr_ex_action sim_world_msgs
)

add_executable(apriltag_sim_node src/main.cpp)
target_link_libraries(apriltag_sim_node apriltag_sim_node_lib)
ament_target_dependencies(apriltag_sim_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs std_srvs amr_ex_action sim_world_msgs
)

install(TARGETS apriltag_sim_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS apriltag_sim_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
