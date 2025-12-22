cmake_minimum_required(VERSION 3.0.2)
project(energy_planner)

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
pkg_check_modules(tinyxml2 REQUIRED tinyxml2)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs sensor_msgs mavros_msgs octomap_msgs 
)

include_directories( 
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}  
  ${EIGEN3_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS} 
)

add_executable(energy_aware_path_planner src/energy_aware_path_planner.cpp)
target_link_libraries(energy_aware_path_planner
  ${catkin_LIBRARIES}
  ${tinyxml2_LIBRARIES}
  octomap)
target_compile_features(energy_aware_path_planner PUBLIC cxx_std_20)
add_dependencies(energy_aware_path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wheelbird_local_planner src/wheelbird_local_planner.cpp)
add_dependencies(wheelbird_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird_local_planner ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

