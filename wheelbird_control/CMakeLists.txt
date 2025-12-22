cmake_minimum_required(VERSION 3.0.2)
project(wheelbird_control)

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
)

find_package(casadi REQUIRED)
find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs visualization_msgs sensor_msgs mavros_msgs  
)

include_directories( 
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(wheelbird1_setpoint src/wheelbird1_setpoint.cpp)
add_dependencies(wheelbird1_setpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird1_setpoint ${catkin_LIBRARIES})

add_executable(wheelbird2_setpoint src/wheelbird2_setpoint.cpp)
add_dependencies(wheelbird2_setpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird2_setpoint ${catkin_LIBRARIES})

add_executable(wheelbird3_setpoint src/wheelbird3_setpoint.cpp)
add_dependencies(wheelbird3_setpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird3_setpoint ${catkin_LIBRARIES})

add_executable(wheelbird4_setpoint src/wheelbird4_setpoint.cpp)
add_dependencies(wheelbird4_setpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird4_setpoint ${catkin_LIBRARIES})

add_executable(wheelbird5_setpoint src/wheelbird5_setpoint.cpp)
add_dependencies(wheelbird5_setpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheelbird5_setpoint ${catkin_LIBRARIES})

add_executable(mpc1_ground src/mpc1_ground.cpp)
add_dependencies(mpc1_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc1_ground ${catkin_LIBRARIES} casadi)

add_executable(mpc2_ground src/mpc2_ground.cpp)
add_dependencies(mpc2_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc2_ground ${catkin_LIBRARIES} casadi)

add_executable(mpc3_ground src/mpc3_ground.cpp)
add_dependencies(mpc3_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc3_ground ${catkin_LIBRARIES} casadi)

add_executable(mpc4_ground src/mpc4_ground.cpp)
add_dependencies(mpc4_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc4_ground ${catkin_LIBRARIES} casadi)

add_executable(mpc5_ground src/mpc5_ground.cpp)
add_dependencies(mpc5_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc5_ground ${catkin_LIBRARIES} casadi)

add_executable(mpc6_ground src/mpc6_ground.cpp)
add_dependencies(mpc6_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc6_ground ${catkin_LIBRARIES} casadi)

add_executable(set_mode src/set_mode.cpp)
add_dependencies(set_mode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(set_mode ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
