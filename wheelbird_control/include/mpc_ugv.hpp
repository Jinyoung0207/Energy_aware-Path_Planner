#pragma once
#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <mutex>
#include <memory>
#include <functional>
#include <experimental/filesystem>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/ColorRGBA.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "ros/ros.h"

#include <casadi/casadi.hpp>


#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <termios.h>
#include <chrono>
#include <thread>
#include <std_msgs/Int64.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <tf/transform_datatypes.h>

using namespace std;

class mpc_ugv : public ros::NodeHandle
{
public:

    void node_param_init();
    void init();
    void run();

private:
    // ROS1
    std_msgs::Int64 mode_num;
    std_msgs::Int64 controller_mode_num;
    ros::Time ros1time;
    ros::NodeHandle nh_;

    ros::Subscriber set_mode_sub;
    ros::Subscriber rbt_odom_sub;
    ros::Subscriber rbt_vel_sub;

    ros::Subscriber rbt_ref_traj_sub;
    /////////////////////충돌회피 부분 추가///////////////////
    ros::Subscriber rbt1_odom_sub;
    ros::Subscriber rbt2_odom_sub;
    ros::Subscriber rbt3_odom_sub;
    ros::Subscriber rbt4_odom_sub;
    ros::Subscriber rbt5_odom_sub;

    ros::Subscriber rbt1_ref_traj_sub;
    ros::Subscriber rbt2_ref_traj_sub;
    ros::Subscriber rbt3_ref_traj_sub;
    ros::Subscriber rbt4_ref_traj_sub;
    ros::Subscriber rbt5_ref_traj_sub;

    ros::Subscriber rbt1_pre_sub;
    ros::Subscriber rbt2_pre_sub;
    ros::Subscriber rbt3_pre_sub;
    ros::Subscriber rbt4_pre_sub;
    ros::Subscriber rbt5_pre_sub;
    ros::Subscriber mode_flag_sub;
    queue<nav_msgs::Path> rbt1_pre_queue;
    queue<nav_msgs::Path> rbt2_pre_queue;
    queue<nav_msgs::Path> rbt3_pre_queue;
    queue<nav_msgs::Path> rbt4_pre_queue;
    queue<nav_msgs::Path> rbt5_pre_queue;
    //////////////////////////
    double cumulative_squared_error = 0.0;
    int sample_count = 0;
    double x_error = 0;
    double y_error = 0;
    double z_error = 0;
    double squared_error = 0;
    double rmse = 0;


    double safe_distance = 5.0;
    double pre_distance;
    double a = 0;
    bool pre_path_in = false;
    bool pre1_path_in = false;
    bool pre2_path_in = false;
    bool pre3_path_in = false;
    bool pre4_path_in = false;
    bool pre5_path_in = false;
        
    casadi::SX init_g;

    /////////////////////충돌회피 부분 추가////////////////////////
    
    ros::Publisher attitude_pub;
    ros::Publisher rbt_cmd_pub;
    ros::Publisher rbt_pre_pub;
    ros::Publisher marker_pub;

    geometry_msgs::PoseStamped rbt_odom_msg;
    nav_msgs::Path rbt_ref_traj_msg;
    geometry_msgs::TwistStamped rbt_vel_msg;

    geometry_msgs::TwistStamped rbt_cmd_msg;
    nav_msgs::Path rbt_pre_msg;
    
    nav_msgs::Path rbt1_pre_msg;
    nav_msgs::Path rbt2_pre_msg;
    nav_msgs::Path rbt3_pre_msg;
    nav_msgs::Path rbt4_pre_msg;
    nav_msgs::Path rbt5_pre_msg;
    mavros_msgs::AttitudeTarget attitude_msg;

    nav_msgs::Path rbt_ref_msg;
    
    queue<geometry_msgs::PoseStamped> rbt_odom_queue;
    queue<geometry_msgs::TwistStamped> rbt_vel_queue;

    queue<nav_msgs::Path> rbt_ref_traj_queue;
    mutex buf;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    bool rbt_odom_in = false;
    bool rbt_vel_in = false;

    bool rbt_ref_traj_in = false;
    bool sub_print_once = false;
    
    bool ugv1_on;
    bool ugv2_on;
    bool ugv3_on;
    bool ugv4_on;
    bool ugv5_on;

    string ref_topic;
    string odom_topic;
    string cmd_topic;
    string pre_topic;
    string robot_name;
    string frame_id;

    // MPC 
    double dt;
    int N;
    

    casadi::SX x;
    casadi::SX y;
    casadi::SX theta;
    casadi::SX states;
    int n_states;

    casadi::SX v;
    casadi::SX omega;
    casadi::SX z_vel;
    casadi::SX controls;
    int n_controls;

    casadi::SX rhs;

    double Q_x     = 0.1;
    double Q_y     = 0.1;
    double Q_theta = 0.01;

    double Q_terminor = 10.0;

    double R1 = 0.005;
    double R2 = 0.005;
    double R3 = 0.005;

    casadi::Function solver;
    casadi::Function f;
    casadi::SX U;
    casadi::SX X;
    casadi::SX P;

    casadi::SX Q;
    casadi::SX R;
    casadi::SX Q_ter;

    casadi::SX g;

    double x_max     =  casadi::inf;
    double x_min     = -casadi::inf;
    double y_max     =  casadi::inf;
    double y_min     = -casadi::inf;
    double theta_max =  casadi::inf;
    double theta_min = -casadi::inf;

    // double v_max     = 0.3;
    double v_max     = 0.0;
    double v_min     = 0.0;
    // double omega_max = M_PI/4.0;
    // double omega_min = -(M_PI/4.0);
    double omega_max = M_PI/4.0;
    double omega_min = -(M_PI/4.0);

    double k_z_yaw_gain    = 0.2;
    double z_vel_max       = 0.1;
    double z_vel_min       = 0.0;

    casadi::DM lbg;
    casadi::DM ubg;
    casadi::DM lbx;
    casadi::DM ubx;

    casadi::DMDict args;

    casadi::DM rbt_state_init;
    casadi::DM X0;
    casadi::DM U0;

    casadi::DMDict solver_result;

    int loop_count = 0;
    vector<vector<double>> obstacle_vec;
    int n_obstacles = 0;
    // loop
    void log_data(double time, double ref_vx, double vx_now, double vx_error,
        double pitch, double thrust, double pitch_est, double thrust_est, double correction);
    void compute_pitch_thrust(double vx_ref, double vx_now, double yaw_error, double& pitch_out, double& thrust_out);
    void mode_num_Callback(const std_msgs::Int64::ConstPtr &msg);
    void rbt_odom_Callbck(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void rbt_vel_Callbck(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void controller_mode_num_Callback(const std_msgs::Int64::ConstPtr &msg);
    void rbt_ref_traj_Callbck(const nav_msgs::Path::ConstPtr& msg);
    
    void rbt1_pre_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt2_pre_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt3_pre_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt4_pre_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt5_pre_Callback(const nav_msgs::Path::ConstPtr& msg);
    
    void collision_avoid();
    
    void rbt1_pre_update();
    void rbt2_pre_update();
    void rbt3_pre_update();
    void rbt4_pre_update();
    void rbt5_pre_update();


    void rbt_odom_update();
    void rbt_vel_update();

    void rbt_ref_traj_update();
    void state_update();
    void setting_solver();
    void make_args_p();
    void setting_reference();
    void reshape_and_init_opt_variable();
    void call_solver();
    void get_result();
    void current_pos_msg_set();
    void cmd_vel_msg_set();
    void predictive_traj_msg_set();
    void publish();
    void shift();
    void reset();
    
    void avoid_obstacle();
    void publish_obstacles();
    void obstacle_set();
    // once
    void mpc_init();
};
