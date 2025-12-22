# Energy Aware Path Planner 🚁🚗

**군집 지상-공중 이중 모드 로봇(Wheelbird)의 에너지 소모량을 고려한 경로 계획 및 제어 패키지
PX4-SITL 기반의 시뮬레이션 환경에서 다중 로봇의 경로를 생성하고 제어합니다.

## 🛠️ Environment (개발 환경)
* **OS:** Ubuntu 20.04 (Focal Fossa)
* **Middleware:** ROS Noetic
* **Simulator:** Gazebo Classic, PX4-SITL
* **Communication:** MAVROS
* **Vehicle:** Wheelbird (Passive wheel mechanism)

---

## 🚀 How to Run (실행 방법)

### 1. Simulation & Map Setup (시뮬레이션 및 맵 로드)
먼저 가제보 시뮬레이터와 RViz를 실행하고, Octomap을 생성합니다.

```bash
# 1. 맵 로드 및 로봇 스폰 (현재 4대 설정)
roslaunch wheelbird_gazebo wheelbird_multi_rviz.launch

# 2. 장애물 맵 생성 (Octomap)
# 주의: 소스코드 내 맵 파일 경로(.bt file)를 본인 환경에 맞게 수정해야 합니다.
rosrun octomap_generator octomap_publish_node

# 3. Energy-aware Global Path Planner 실행
roslaunch energy_planner energy_aware_path_planner.launch

# 4. Local Path Planner 실행
roslaunch energy_planner wheelbird_local_planner.launch

# 5. 지상 주행 제어 (Ground Control - MPC)
# n은 로봇 번호에 맞게 변경 (예: mpc1_ground.launch)
roslaunch wheelbird_control mpc(n)_ground.launch

# 6. 공중 비행 제어 (Aerial Control)
rosrun wheelbird_control wheelbird1_setpoint
