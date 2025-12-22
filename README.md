# Energy Aware Path Planner
This is a ROS package for bimodal robot path planning and control.
군집 지상-공중 이중 모드 로봇의 에너지 소모량을 고려한 경로계획 및 제어(Simulation Environment: PX4-SITL, MAVROS, Ubuntu20.04, Gazebo-Classic)
used vehicle: Wheelbird(Passive wheel)

////////////////////////////////////////맵 & 로봇 스폰 및 경로계획 관련///////////////////////////////////////////////
1. Map & Robot Spawn: roslaunch wheelbird_gazebo wheelbird_multi_rviz.launch (현재 4대)
2. Obstacle map generator: rosrun octomap_generator octomap_publish_node (사용할 Map 파일 본인 경로 맞게 지정 필요)
3. Energy-aware Path Planner: roslaunch energy_planner energy_aware_path_planner.launch
4. Local path planner: roslaunch energy_planner wheelbird_local_planner.launch
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////// 로봇 지상 & 공중 제어 관련///////////////////////////////////////////////
1. roslaunch wheelbird_control mpc(n)_ground.launch (지상 제어 노드)
2. rosrun wheelbird_control wheelbird1_setpoint (공중 제어 노드)
