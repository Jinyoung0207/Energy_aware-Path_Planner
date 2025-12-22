#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cmath>
#include <queue>
#include <mutex>
#include <numeric>
#include <experimental/filesystem>

#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Path.h> 
#include <geometry_msgs/PointStamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ParamSet.h>

#include <termios.h>
#include <chrono>
#include <thread>

class multi_setpoint_ctrl
{
public:
    multi_setpoint_ctrl(ros::NodeHandle& nh)
    {}
    ~multi_setpoint_ctrl()
    {}

    void init();
    void run();

private:
    // ROS 관련 멤버 변수
    ros::NodeHandle nh;

    double cumulative_squared_error = 0.0;
    int sample_count = 0;
    double x_error = 0;
    double y_error = 0;
    double z_error = 0;
    double squared_error = 0;
    double rmse = 0;
    
    ros::Publisher set_point_1_pub;
    ros::Publisher set_point_2_pub;
    ros::Publisher set_point_3_pub;
    ros::Publisher set_point_4_pub;
    ros::Publisher set_point_5_pub;

    ros::Publisher UAV1_alt_check_pub;
    ros::Publisher UAV2_alt_check_pub;
    ros::Publisher UAV3_alt_check_pub;
    ros::Publisher UAV4_alt_check_pub;
    ros::Publisher UAV5_alt_check_pub;

    ros::Publisher error1_pub;
    ros::Publisher error2_pub;
    ros::Publisher error3_pub;
    ros::Publisher error4_pub;
    ros::Publisher error5_pub;

    ros::Publisher avp1_pub;
    ros::Publisher initial_yaw_1_pub;

    ros::Publisher path_1_pub , path_2_pub , path_3_pub ,path_4_pub ,path_5_pub;
    ros::Publisher pose_point_1_pub, pose_point_2_pub ,pose_point_3_pub ,pose_point_4_pub ,pose_point_5_pub; 

    ros::Subscriber mode_flag_sub;
    ros::Subscriber current_pose_1_sub;
    ros::Subscriber current_pose_2_sub;
    ros::Subscriber current_pose_3_sub;
    ros::Subscriber current_pose_4_sub;
    ros::Subscriber current_pose_5_sub;

    ros::Subscriber ref_1_sub;
    ros::Subscriber ref_2_sub;
    ros::Subscriber ref_3_sub;
    ros::Subscriber ref_4_sub;
    ros::Subscriber ref_5_sub;

    ros::Subscriber initial_yaw_sub;
    ros::Subscriber initial_pose1_sub;
    ros::Subscriber initial_pose2_sub;
    ros::Subscriber initial_pose3_sub;
    ros::Subscriber initial_pose4_sub;
    ros::Subscriber initial_pose5_sub;

    ros::Subscriber state1_sub;
    ros::Subscriber state2_sub;
    ros::Subscriber state3_sub;
    ros::Subscriber state4_sub;
    ros::Subscriber state5_sub;

    ros::Subscriber set_mode_sub;
    ros::Subscriber th_sub;
    ros::Subscriber land_vel_sub;

    ros::ServiceClient arming_client1;
    ros::ServiceClient set_mode_client1;
    ros::ServiceClient arming_client2;
    ros::ServiceClient set_mode_client2;
    ros::ServiceClient arming_client3;
    ros::ServiceClient set_mode_client3;
    ros::ServiceClient arming_client4;
    ros::ServiceClient set_mode_client4;
    ros::ServiceClient arming_client5;
    ros::ServiceClient set_mode_client5;

    ros::ServiceClient param_set_client1;
    ros::ServiceClient param_th_client1;
    ros::ServiceClient param_set_client2;
    ros::ServiceClient param_th_client2;
    ros::ServiceClient param_set_client3;
    ros::ServiceClient param_th_client3;
    ros::ServiceClient param_set_client4;
    ros::ServiceClient param_th_client4;
    ros::ServiceClient param_set_client5;
    ros::ServiceClient param_th_client5;

    mavros_msgs::State current_state1;
    mavros_msgs::State current_state2;
    mavros_msgs::State current_state3;
    mavros_msgs::State current_state4;
    mavros_msgs::State current_state5;

    // Buffer와 플래그
    std::queue<nav_msgs::Path> ref_1_queue;
    std::queue<nav_msgs::Path> ref_2_queue;
    std::queue<nav_msgs::Path> ref_3_queue;
    std::queue<nav_msgs::Path> ref_4_queue;
    std::queue<nav_msgs::Path> ref_5_queue;
    std::mutex buf1;
    std::mutex buf2;
    std::mutex buf3;
    std::mutex buf4;
    std::mutex buf5;

    bool ref_traj_1_in = false;
    bool ref_traj_2_in = false;
    bool ref_traj_3_in = false;
    bool ref_traj_4_in = false;
    bool ref_traj_5_in = false;

    double initial_yaw_degree;
    std_msgs::Float32 yaw;

    double UAV1_init[2];
    double UAV2_init[2];
    double UAV3_init[2];
    double UAV4_init[2];
    double UAV5_init[2];

    int wait_time1=0;
    int wait_time2=0;
    int wait_time3=0;
    int wait_time4=0;
    int wait_time5=0;

    int land_time=0;

    int UAV1_alt=0;
    int UAV2_alt=0;
    int UAV3_alt=0;
    int UAV4_alt=0;
    int UAV5_alt=0;

    double h1=0.42;
    double h2=0.42;
    double h3=0.42;
    double h4=0.42;
    double h5=0.42;

    double land_alt1=0.1;

    bool arm_check1=true;
    bool mode_check1=true;
    bool land_check1=true;
    bool arm_check2=true;
    bool mode_check2=true;
    bool land_check2=true;
    bool arm_check3=true;
    bool mode_check3=true;
    bool land_check3=true;
    bool arm_check4=true;
    bool mode_check4=true;
    bool land_check4=true;
    bool arm_check5=true;
    bool mode_check5=true;
    bool land_check5=true;

    std_msgs::Float32 UAV1_error;
    std_msgs::Float32 UAV2_error;
    std_msgs::Float32 UAV3_error;
    std_msgs::Float32 UAV4_error;
    std_msgs::Float32 UAV5_error;

    std_msgs::Float32 initial_yaw;
    std_msgs::Int64 mode_num;
    std_msgs::Int64 controller_mode_num;
    std_msgs::Float32 th; 
    std_msgs::Float32 land_vel;

    double error1;
    double error2;
    double error3;
    double error4;
    double error5;

    int avp1_done=0;
    int avp2_done=0;
    int avp3_done=0;
    int avp4_done=0;
    int avp5_done=0;

    // 메시지 타입
    
    geometry_msgs::PoseStamped UAV1_pose;
    geometry_msgs::PoseStamped UAV2_pose;
    geometry_msgs::PoseStamped UAV3_pose;
    geometry_msgs::PoseStamped UAV4_pose;
    geometry_msgs::PoseStamped UAV5_pose;

    geometry_msgs::PoseStamped UAV1_initial_pose;
    geometry_msgs::PoseStamped UAV2_initial_pose;
    geometry_msgs::PoseStamped UAV3_initial_pose;
    geometry_msgs::PoseStamped UAV4_initial_pose;
    geometry_msgs::PoseStamped UAV5_initial_pose;

    geometry_msgs::PoseStamped avp1;
    geometry_msgs::PoseStamped avp2;
    geometry_msgs::PoseStamped avp3;
    geometry_msgs::PoseStamped avp4;
    geometry_msgs::PoseStamped avp5;

    nav_msgs::Path ref_path_1_msg;
    nav_msgs::Path ref_path_2_msg;
    nav_msgs::Path ref_path_3_msg;
    nav_msgs::Path ref_path_4_msg;
    nav_msgs::Path ref_path_5_msg;

    geometry_msgs::PoseStamped sp1;
    geometry_msgs::PoseStamped sp2;
    geometry_msgs::PoseStamped sp3;
    geometry_msgs::PoseStamped sp4;
    geometry_msgs::PoseStamped sp5;
    std_msgs::Int64 UAV1_alt_check;
    std_msgs::Int64 UAV2_alt_check;
    std_msgs::Int64 UAV3_alt_check;
    std_msgs::Int64 UAV4_alt_check;
    std_msgs::Int64 UAV5_alt_check;

    nav_msgs::Path UAV1_path;
    nav_msgs::Path UAV2_path;
    nav_msgs::Path UAV3_path;
    nav_msgs::Path UAV4_path;
    nav_msgs::Path UAV5_path; 
    geometry_msgs::PoseStamped path_1,path_2,path_3,path_4,path_5;

    geometry_msgs::PointStamped UAV1_point;
    geometry_msgs::PointStamped UAV2_point;
    geometry_msgs::PointStamped UAV3_point;
    geometry_msgs::PointStamped UAV4_point;
    geometry_msgs::PointStamped UAV5_point;

    geometry_msgs::PoseStamped UAV1_landing_pose;

    // 콜백 함수들
    // void formation_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pose4Callback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void pose5Callback(const geometry_msgs::PoseStamped::ConstPtr& msg); 
    void initial_pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initial_pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initial_pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initial_pose4Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initial_pose5Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void ref_1_Callback(const nav_msgs::Path::ConstPtr &msg);
    void ref_2_Callback(const nav_msgs::Path::ConstPtr &msg);
    void ref_3_Callback(const nav_msgs::Path::ConstPtr &msg);
    void ref_4_Callback(const nav_msgs::Path::ConstPtr &msg);
    void ref_5_Callback(const nav_msgs::Path::ConstPtr &msg);
    void initial_yaw_Callback(const std_msgs::Float32::ConstPtr &msg);

    void state1Callback(const mavros_msgs::State::ConstPtr &msg); 
    void state2Callback(const mavros_msgs::State::ConstPtr &msg); 
    void state3Callback(const mavros_msgs::State::ConstPtr &msg); 
    void state4Callback(const mavros_msgs::State::ConstPtr &msg); 
    void state5Callback(const mavros_msgs::State::ConstPtr &msg); 
    
    void controller_mode_num_Callback(const std_msgs::Int64::ConstPtr &msg);

    void mode_num_Callback(const std_msgs::Int64::ConstPtr &msg);
    void th_Callback(const std_msgs::Float32::ConstPtr &msg);
    void land_vel_Callback(const std_msgs::Float32::ConstPtr &msg);
    
    // 유틸리티 함수들
    void ref_path_update();
    void averaging_pose();
    void altitude_check();
    void set_point();
    void publishAll();
    void set_initial_orientation();
    double yaw_from_quaternion(double x, double y, double z, double w);
    void mode_arming();
    void mode_offboard();
    void mode_landing();
    void mode_autoland();
};
