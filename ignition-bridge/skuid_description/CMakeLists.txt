cmake_minimum_required(VERSION 3.8)
project(skuid_description)

# IF NEEDED TO MONITOR DDS
set(FASTDDS_STATISTICS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

add_library(tf_start_base_link SHARED src/tf2_base_link_pub.cpp)

target_include_directories(tf_start_base_link PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# add_executable(fastdds_test_node_A src/fastdds_test_node_A.cpp)

# target_include_directories(fastdds_test_node_A PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_link_directories(fastdds_test_node_A PUBLIC /opt/vulcanexus/galactic/lib)

# target_link_libraries(fastdds_test_node_A fastrtps rmw_fastrtps_cpp)

# add_executable(fastdds_test_node_B src/fastdds_test_node_B.cpp)

# target_include_directories(fastdds_test_node_B PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)


ament_target_dependencies(tf_start_base_link
"rclcpp"
"geometry_msgs"
"rclcpp_components"
"tf2_ros"
"tf2"
)

# ament_target_dependencies(fastdds_test_node_A
# "rclcpp"
# "std_msgs"
# )

# ament_target_dependencies(fastdds_test_node_B
# "rclcpp"
# "std_msgs"
# )

rclcpp_components_register_node(tf_start_base_link PLUGIN "skuid::ign_localization_node" EXECUTABLE tf_start_base_link_exec)

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
tf_start_base_link
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)


# install(TARGETS 
# fastdds_test_node_A
# fastdds_test_node_B
# DESTINATION lib/${PROJECT_NAME})


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
