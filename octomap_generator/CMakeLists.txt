cmake_minimum_required(VERSION 3.0.2)
project(octomap_generator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
  mavros_msgs
  octomap_msgs
  message_generation
)

find_package(octomap REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs sensor_msgs mavros_msgs octomap_msgs 
)

include_directories( 
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS} 
)

add_executable(octomap_publish_node src/octomap_publish_node.cpp)
add_dependencies(octomap_publish_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_publish_node   
  ${catkin_LIBRARIES}
  ${tinyxml2_LIBRARIES}  
  octomap
)
