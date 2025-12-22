#include "mpc_ugv.hpp"

void mpc_ugv::mode_num_Callback(const std_msgs::Int64::ConstPtr &msg)
{
    mode_num=*msg;
}

void mpc_ugv::controller_mode_num_Callback(const std_msgs::Int64::ConstPtr &msg)
{
    controller_mode_num=*msg;
    std::cout << "controller_mode_num: " << controller_mode_num.data << std::endl;
}

double pi_to_pi(double angle)
{
    while(angle >= M_PI)
        angle -= 2.0 * M_PI;

    while(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

double q_to_e(geometry_msgs::PoseStamped msg)
{
    tf::Quaternion q;
    q.setX(msg.pose.orientation.x);
    q.setY(msg.pose.orientation.y);
    q.setZ(msg.pose.orientation.z);
    q.setW(msg.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

tf::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    // q.normalize(); // Uncomment if normalization is needed
    return q;
}

casadi::SX shortest_angle(const casadi::SX& current_angle, const casadi::SX& target_angle)
{
    casadi::SX diff = target_angle - current_angle;
    
    casadi::SX normalized = casadi::SX::atan2(casadi::SX::sin(diff), 
                                              casadi::SX::cos(diff));
    
    return casadi::SX::abs(normalized);
}

void mpc_ugv::setting_reference()
{
    double theta_ref_prev = 0.0;
    
    for(int n = 0; n < N; ++n)
    {
        double theta_ref;
        double u_ref;
        double omega_ref = 0.0;
        
        if (n < N - 1) 
        {
            const auto& pt_now = rbt_ref_traj_msg.poses[n].pose.position;
            const auto& pt_next = rbt_ref_traj_msg.poses[n+1].pose.position;
            u_ref = sqrt(pow(pt_next.x - pt_now.x, 2) + pow(pt_next.y - pt_now.y, 2)) / dt;
            theta_ref = atan2(pt_next.y - pt_now.y, pt_next.x - pt_now.x);
        } 
        else 
        {
            const auto& pt_now = rbt_ref_traj_msg.poses[n].pose.position;
            const auto& pt_prev = rbt_ref_traj_msg.poses[n-1].pose.position;
            u_ref = sqrt(pow(pt_now.x - pt_prev.x, 2) + pow(pt_now.y - pt_prev.y, 2)) / dt;
            theta_ref = atan2(pt_now.y - pt_prev.y, pt_now.x - pt_prev.x);
        }
        
        if(n > 0)
        {
            omega_ref = pi_to_pi(theta_ref - theta_ref_prev) / dt;
        }
        theta_ref_prev = theta_ref;

        args["p"]((n_states + n_controls) * n + 3) = rbt_ref_traj_msg.poses[n].pose.position.x;
        args["p"]((n_states + n_controls) * n + 4) = rbt_ref_traj_msg.poses[n].pose.position.y;
        args["p"]((n_states + n_controls) * n + 5) = theta_ref;
        args["p"]((n_states + n_controls) * n + 6) = u_ref;
        args["p"]((n_states + n_controls) * n + 7) = omega_ref;
    }
}
void mpc_ugv::make_args_p()
{
    casadi::DM p = casadi::DM::zeros((n_states + n_controls) * N + n_states,1);

    args.insert(std::pair<std::string,casadi::DM>("p",p));

    for(int idx = 0; idx < n_states; ++idx)
    {
        args["p"](idx) = rbt_state_init(idx);
    }
}
void mpc_ugv::rbt_vel_Callbck(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    buf.lock();
    rbt_vel_queue.push(*msg);
    buf.unlock();
}

void mpc_ugv::rbt_odom_Callbck(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    buf.lock();
    rbt_odom_queue.push(*msg);
    buf.unlock();
}

void mpc_ugv::rbt_ref_traj_Callbck(const nav_msgs::Path::ConstPtr& msg)
{
    buf.lock();
    rbt_ref_traj_queue.push(*msg);
    buf.unlock();
}

void mpc_ugv::reset()
{
    rbt_pre_msg.poses.clear();
    rbt_ref_msg.poses.clear();

    ++loop_count;

    rbt_odom_in = false;
    rbt_vel_in = false;
    rbt_ref_traj_in = false;
}

void mpc_ugv::compute_pitch_thrust(double vx_ref,
                                   double vx_now,
                                   double yaw_error,
                                   double& pitch_out,
                                   double& thrust_out)
{
    const double vx_max     = 0.8;
    const double pitch_max  = 0.4;    // ≃23°
    const double thrust_max = 0.4;
    const double g          = 9.81;

    // 1) Feed-forward table (unchanged)
    struct PT { double vx, pitch, thrust; };
    static std::vector<PT> table = 
    {
        {0.00, 0.0000, 0.10},
        {0.10, 0.1396, 0.10},
        {0.15, 0.1993, 0.10},
        {0.20, 0.1993, 0.15},
        {0.35, 0.1993, 0.20},
        {0.45, 0.2978, 0.20},
        {0.60, 0.2978, 0.25},
        {0.70, 0.3373, 0.25},
        {0.80, 0.3948, 0.25}
    };

    // 2) 속도 저역통과 필터
    static double vx_filt = 0.0;
    const double alpha = 0.2;
    vx_filt = alpha * vx_now + (1.0 - alpha) * vx_filt;

    // 3) 테이블 보간: feed-forward
    double vx_clamped = std::clamp(vx_ref, 0.0, vx_max);
    double pitch_ff = 0.0, thrust_ff = 0.0;
    for (size_t i = 0; i + 1 < table.size(); ++i) {
      if (vx_clamped >= table[i].vx && vx_clamped <= table[i+1].vx) {
        double r = (vx_clamped - table[i].vx)
                 / (table[i+1].vx - table[i].vx);
        pitch_ff  = table[i].pitch  + r * (table[i+1].pitch  - table[i].pitch);
        thrust_ff = table[i].thrust + r * (table[i+1].thrust - table[i].thrust);
        break;
      }
    }

    // 4) PID → 원하는 가속도 a_des
    static double integral = 0.0;
    static double prev_err = 0.0;
    static ros::Time last_t = ros::Time::now();
    ros::Time now = ros::Time::now();
    double dt = (now - last_t).toSec();
    last_t = now;

    double err = vx_ref - vx_filt;
    integral += err * dt;
    integral = std::clamp(integral, -5.0, 5.0);
    double derivative = (err - prev_err) / dt;
    prev_err = err;

    const double Kp = 1.0, Ki = 0.2, Kd = 0.1;
    double a_des = Kp * err + Ki * integral + Kd * derivative;

    // 5) 가속도 → 피치 보정: a = g * tan(theta)
    double pitch_corr = std::atan2(a_des, g);

    // 6) 최종 피치 & thrust
    pitch_out  = std::clamp(pitch_ff + pitch_corr, 0.0, pitch_max);
    thrust_out = std::clamp(thrust_ff, 0.1, thrust_max);

    ROS_INFO_STREAM("[compute] vx_ref="  << vx_ref
                    << "  vx_filt="    << vx_filt
                    << "  err="        << err
                    << "  a_des="      << a_des
                    << "  pitch_ff="   << pitch_ff
                    << "  pitch_corr=" << pitch_corr
                    << "  pitch_out="  << pitch_out
                    << "  thrust_out=" << thrust_out);
}


void mpc_ugv::publish()
{
    rbt_pre_pub.publish(rbt_pre_msg);
    attitude_pub.publish(attitude_msg);
    
    x_error = rbt_odom_msg.pose.position.x - rbt_ref_traj_msg.poses[0].pose.position.x;
    y_error = rbt_odom_msg.pose.position.y - rbt_ref_traj_msg.poses[0].pose.position.y;
    z_error = rbt_odom_msg.pose.position.z - rbt_ref_traj_msg.poses[0].pose.position.z;
    
    squared_error = x_error * x_error + y_error * y_error + z_error * z_error;
    
    cumulative_squared_error += squared_error;
    
    sample_count++;
    
    rmse = std::sqrt(cumulative_squared_error / sample_count);

    // std::cout << "rmse result is : " << rmse << "sample_count_is: " << sample_count << "cumulative_squared_error_is: " << cumulative_squared_error << std::endl;
    x_error = 0;
    y_error = 0;
    z_error = 0;
}

void mpc_ugv::cmd_vel_msg_set()
{
    rbt_cmd_msg.twist.linear.x = (double)U0(0,0);
    rbt_cmd_msg.twist.angular.z = (double)U0(1,0);

    double vx_ref = (double)U0(0,0);
    double vx_now = std::sqrt(
        std::pow(rbt_vel_msg.twist.linear.x, 2) +
        std::pow(rbt_vel_msg.twist.linear.y, 2) 
    );
    
    double pitch, thrust;
    double yaw = 0.0;

    if(rbt_pre_msg.poses.size() > 0)
    {
        int lookahead_idx = 10;
        yaw = q_to_e(rbt_pre_msg.poses[lookahead_idx]);
    }
    
    compute_pitch_thrust(vx_ref, vx_now, yaw, pitch, thrust);

    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, pitch, yaw);
    geometry_msgs::Quaternion q_msg;
    tf::quaternionTFToMsg(q, q_msg);

    attitude_msg.header.stamp = ros::Time::now();
    attitude_msg.type_mask = 
        mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
        mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

    attitude_msg.orientation = q_msg;
    attitude_msg.body_rate.x = 0.0;
    attitude_msg.body_rate.y = 0.0;
    attitude_msg.body_rate.z = 0.0;
    attitude_msg.thrust = thrust;
}

void mpc_ugv::predictive_traj_msg_set()
{
    rbt_pre_msg.header.frame_id = "map";
    rbt_pre_msg.header.stamp = ros::Time::now();
    for(int n = 0; n < N; ++n)
    {
        geometry_msgs::PoseStamped n_path;
        n_path.pose.position.x = (double)X0(0,n);
        n_path.pose.position.y = (double)X0(1,n);

        tf::Quaternion n_q;
        n_q.setRPY(0.0, 0.0, pi_to_pi((double)X0(2,n)));

        n_path.pose.orientation.x = n_q.x();
        n_path.pose.orientation.y = n_q.y();
        n_path.pose.orientation.z = n_q.z();
        n_path.pose.orientation.w = n_q.w();

        rbt_pre_msg.poses.push_back(n_path);
    }
}

void mpc_ugv::shift()
{
    casadi::DM X0_last = casadi::DM::vertcat({X0(0,-1),X0(1,-1),X0(2,-1)});
    X0 = casadi::DM::reshape(X0,X0.size1()*X0.size2(),1);
    X0 = casadi::DM::vertcat({X0(casadi::Slice(n_states,X0.size1()*X0.size2())),X0_last});
    X0 = casadi::DM::reshape(X0,n_states,N+1);

    casadi::DM U0_last = casadi::DM::vertcat({U0(0,-1),U0(1,-1)});
    U0 = casadi::DM::reshape(U0,U0.size1()*U0.size2(),1);
    U0 = casadi::DM::vertcat({U0(casadi::Slice(n_controls,U0.size1()*U0.size2())),U0_last});

    U0 = casadi::DM::reshape(U0,n_controls,N);
}

void mpc_ugv::get_result()
{
    casadi::Slice slice_x0(0,n_states*(N+1));
    casadi::DM X0_temp = solver_result["x"](slice_x0);
    X0 = casadi::DM::reshape(X0_temp,n_states,N+1);

    casadi::Slice slice_u0(n_states*(N+1),solver_result["x"].size1());
    casadi::DM U0_temp = solver_result["x"](slice_u0);
    U0 = casadi::DM::reshape(U0_temp,n_controls,N);
}

void mpc_ugv::call_solver()
{
    solver_result = solver(args);
}

void mpc_ugv::reshape_and_init_opt_variable()
{
    args["x0"] = casadi::DM::vertcat({casadi::DM::reshape(X0,n_states * (N+1),1) , 
                                                  casadi::DM::reshape(U0,n_controls * N,1)});
}

void mpc_ugv::setting_solver()
{
    casadi::SX obj = 0;
    casadi::SXVector g_vec;
    for(int i = 0; i < n_states; ++i)
    {
        g_vec.emplace_back(X(i,0) - P(i));
        //std::cout << " value : " << X(i,0) - P(i) << std::endl;
    }
    g = casadi::SX::vertcat(g_vec);

    for(int n = 0; n < N; ++n)
    {
        casadi::SXVector st_vec;
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n));
        }
        casadi::SX st = casadi::SX::vertcat(st_vec);

        casadi::SXVector con_vec;
        for(int idx = 0; idx < n_controls; ++idx)
        {
            con_vec.emplace_back(U(idx,n));
        }
        casadi::SX con = casadi::SX::vertcat(con_vec);

        // casadi::SXVector p_vec;
        // for(int idx = 0; idx < n_states; ++idx)
        // {
        //     p_vec.emplace_back(st(idx) - P((n_states + n_controls) * n + 3 + idx,0));
        // }

        casadi::SXVector p_vec;
        for (int idx = 0; idx < n_states; ++idx)
        {
            if (idx == 2)
            {
                casadi::SX target_yaw = P((n_states + n_controls) * n + 3 + idx, 0);
                casadi::SX angle_error = shortest_angle(st(idx), target_yaw);
                p_vec.emplace_back(angle_error);
            }
            else
            {
                p_vec.emplace_back(st(idx) - P((n_states + n_controls) * n + 3 + idx, 0));
            }
        }
        casadi::SX state_err = casadi::SX::vertcat(p_vec);

        p_vec.clear();
        for(int idx = 0; idx < n_controls; ++idx)
        {
            p_vec.emplace_back(con(idx) - P((n_states + n_controls) * n + 6 + idx,0));
        }
        casadi::SX con_err = casadi::SX::vertcat(p_vec);

        if(n >= N - 1)
        {
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err) + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err);
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q_ter),state_err);
        }
        else
        {
            obj = obj + casadi::SX::mtimes(casadi::SX::mtimes(state_err.T(),Q),state_err)*dt + casadi::SX::mtimes(casadi::SX::mtimes(con_err.T(),R),con_err)*dt;
        }

        st_vec.clear();
        for(int idx = 0; idx < n_states; ++idx)
        {
            st_vec.emplace_back(X(idx,n+1));
        }
        casadi::SX st_next = casadi::SX::vertcat(st_vec);
        casadi::SXVector f_value_vec = f(casadi::SXVector{st,con});
        casadi::SX f_value = casadi::SX::vertcat(f_value_vec);
        casadi::SX st_next_euler = st + (dt * f_value);
        g = casadi::SX::vertcat({g,st_next - st_next_euler});
    }

    casadi::SX OPT_variables = casadi::SX::vertcat({casadi::SX::reshape(X,-1,1),casadi::SX::reshape(U,-1,1)});
    casadi::SXDict nlp_prob;

    nlp_prob.insert(std::pair<std::string,casadi::SX>("f",obj));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("x",OPT_variables));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("g",g));
    nlp_prob.insert(std::pair<std::string,casadi::SX>("p",P));

    casadi::Dict opts = {
    {"ipopt", casadi::Dict{
        {"max_iter", 200},
        {"print_level", 0},
        {"acceptable_tol", 1e-8},
        {"acceptable_obj_change_tol", 1e-6}
    }},
    {"print_time", 0}
    };
    solver = casadi::nlpsol("solver","ipopt",nlp_prob,opts);
}

void mpc_ugv::state_update()
{
    double rbt_yaw = pi_to_pi(q_to_e(rbt_odom_msg));
    rbt_state_init = casadi::DM::vertcat({rbt_odom_msg.pose.position.x, rbt_odom_msg.pose.position.y, rbt_yaw});
    std::cout << " X : " << rbt_odom_msg.pose.position.x << std::endl;
    std::cout << " Y : " << rbt_odom_msg.pose.position.y << std::endl;
    std::cout << " Theta : " << rbt_yaw << std::endl;
    std::cout << " ========= " << std::endl;
    double pose_error = sqrt(pow(rbt_odom_msg.pose.position.x - rbt_ref_traj_msg.poses[0].pose.position.x, 2) + \
        pow(rbt_odom_msg.pose.position.y - rbt_ref_traj_msg.poses[0].pose.position.y, 2));
    ROS_INFO_STREAM("pose_error: " << pose_error);
}

void mpc_ugv::rbt_ref_traj_update()
{
    if (!rbt_ref_traj_queue.empty())
    {
        rbt_ref_traj_msg = rbt_ref_traj_queue.front();
        rbt_ref_traj_queue.pop();
        rbt_ref_traj_in = true;
    }
}
void mpc_ugv::rbt_vel_update()
{
    if (!rbt_vel_queue.empty())
    {
        rbt_vel_msg = rbt_vel_queue.front();
        rbt_vel_queue.pop();
        rbt_vel_in = true;
    }
}

void mpc_ugv::rbt_odom_update()
{
    if (!rbt_odom_queue.empty())
    {
        rbt_odom_msg = rbt_odom_queue.front();
        rbt_odom_queue.pop();
        rbt_odom_in = true;
    }
}

void mpc_ugv::mpc_init()
{
    x      = casadi::SX::sym("x");
    y      = casadi::SX::sym("y");
    theta  = casadi::SX::sym("theta");
    states = casadi::SX::vertcat({x, y, theta});
    n_states = states.numel();

    v        = casadi::SX::sym("v");
    omega    = casadi::SX::sym("omega");
    controls = casadi::SX::vertcat({v, omega});
    n_controls = controls.numel();

    rhs = casadi::SX::vertcat({v * casadi::SX::cos(theta),
                           v * casadi::SX::sin(theta),
                           omega});

    f = casadi::Function("f", {states, controls}, {rhs});
    U = casadi::SX::sym("U", n_controls, N);
    X = casadi::SX::sym("X", n_states, N + 1);
    P = casadi::SX::sym("P", n_states + N * (n_states + n_controls));

    Q = casadi::SX::diagcat({Q_x, Q_y, Q_theta});
    R = casadi::SX::diagcat({R1, R2});

    // Terminal
    Q_ter = casadi::SX::diagcat({Q_x * Q_terminor, Q_y * Q_terminor, Q_theta * Q_terminor});

    lbg = casadi::DM::zeros(n_states * (N + 1), 1);
    ubg = casadi::DM::zeros(n_states * (N + 1), 1);

    lbx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);
    ubx = casadi::DM::zeros(n_states * (N + 1) + n_controls * N, 1);

    for (int idx = 0; idx < n_states * (N + 1); ++idx)
    {
        if (idx % n_states == 0)
        {
            lbx(idx) = x_min;
            ubx(idx) = x_max;
        }
        else if (idx % n_states == 1)
        {
            lbx(idx) = y_min;
            ubx(idx) = y_max;
        }
        else if (idx % n_states == 2)
        {
            lbx(idx) = theta_min;
            ubx(idx) = theta_max;
        }
    }

    bool idx_even = (N + 1) % 2 == 0;

    for (int idx = n_states * (N + 1); idx < lbx.size1(); ++idx)
    {
        if (idx_even)
        {
            if (idx % n_controls == 0)
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
            else
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
        }
        else
        {
            if (idx % n_controls == 0)
            {
                lbx(idx) = omega_min;
                ubx(idx) = omega_max;
            }
            else
            {
                lbx(idx) = v_min;
                ubx(idx) = v_max;
            }
        }
    }

    args.insert(std::make_pair("lbg", lbg));
    args.insert(std::make_pair("ubg", ubg));
    args.insert(std::make_pair("lbx", lbx));
    args.insert(std::make_pair("ubx", ubx));

    rbt_state_init = casadi::DM::vertcat({0.0, 0.0, 0.0});
    U0 = casadi::DM::zeros(N, n_controls);
    X0 = casadi::DM::repmat(rbt_state_init, 1, N + 1);
}

void mpc_ugv::node_param_init()
{
    //int N;
    //double dt, v_max, v_min;
    std::string robot_name_, frame_id_name, ref_topic_name, odom_topic_name, cmd_topic_name, pre_topic_name;
    nh_.getParam("/robot_name", robot_name_);
    nh_.getParam("/frame_id", frame_id_name);
    nh_.getParam("/N", N);
    nh_.getParam("/dt", dt);
    nh_.getParam("/v_max", v_max);
    nh_.getParam("/v_min", v_min);
    nh_.getParam("/omega_max", omega_max);
    nh_.getParam("/cmd_topic_name", cmd_topic_name);
    nh_.getParam("/pose_topic_name", odom_topic_name);
    nh_.getParam("/pre_topic_name", pre_topic_name);
    nh_.getParam("/ref_topic_name", ref_topic_name);
    nh_.getParam("/Q_x", Q_x);
    nh_.getParam("/Q_y", Q_y);
    nh_.getParam("/Q_theta", Q_theta);
    nh_.getParam("/R_v", R1);
    nh_.getParam("/R_omega", R2);    

    robot_name = robot_name_;
    frame_id = frame_id_name;
    N = N;
    dt = dt;
    v_max = v_max;
    v_min = v_min;
    ref_topic = "/" + robot_name + ref_topic_name;
    odom_topic = "/" + robot_name + odom_topic_name;
    cmd_topic = "/" + robot_name + cmd_topic_name;
    pre_topic = "/" + robot_name + pre_topic_name;

    std::cout << "// ================================================ //" << std::endl;
    std::cout << "// Node            : " << N << std::endl;
    std::cout << "// dt              : " << dt << std::endl;
    std::cout << "// v_max           : " << v_max << std::endl;
    std::cout << "// v_min           : " << v_min << std::endl;
    std::cout << "// ref_topic_name  : " << ref_topic << std::endl;
    std::cout << "// pose_topic_name : " << odom_topic << std::endl;
    std::cout << "// cmd_topic_name  : " << cmd_topic << std::endl;
    std::cout << "// pre_topic_name  : " << pre_topic << std::endl;
    std::cout << "// ================================================ //" << std::endl;
}

void mpc_ugv::init()
{
    mode_flag_sub = nh_.subscribe<std_msgs::Int64>("/wheelbird6/mode_flag", 1, &mpc_ugv::controller_mode_num_Callback, this);
    rbt_odom_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/wheelbird6/mavros/local_position/pose", 1, &mpc_ugv::rbt_odom_Callbck, this);
    rbt_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/wheelbird6/mavros/local_position/velocity_local", 1, &mpc_ugv::rbt_vel_Callbck, this);
    rbt_ref_traj_sub = nh_.subscribe<nav_msgs::Path>("/wheelbird6/local_path", 1, &mpc_ugv::rbt_ref_traj_Callbck, this);
    set_mode_sub = nh_.subscribe<std_msgs::Int64>("/mode_num", 1, &mpc_ugv::mode_num_Callback, this);
    // rbt_cmd_pub = nh_.advertise<geometry_msgs::TwistStamped>("/wheelbird/mavros/setpoint_velocity/cmd_vel", 1);
    rbt_pre_pub = nh_.advertise<nav_msgs::Path>(pre_topic, 1);
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
    attitude_pub = nh_.advertise<mavros_msgs::AttitudeTarget>("/wheelbird6/mavros/setpoint_raw/attitude", 1);
}

void mpc_ugv::run()
{

    mpc_init();

    ros::Rate r(1/dt);
    while (ros::ok())
    {
        ros::spinOnce();
        rbt_odom_update();
        rbt_vel_update();
        rbt_ref_traj_update();

        if (rbt_odom_in && rbt_ref_traj_in)
        {
            state_update();
            setting_solver();
            make_args_p();
            setting_reference();
            reshape_and_init_opt_variable();
            call_solver();
            get_result();
            predictive_traj_msg_set();
            cmd_vel_msg_set();
            if(controller_mode_num.data == 0)
            {
                publish();
            }    
            shift();
            reset();
            casadi::Dict solver_iter = solver.stats();
            std::cout << "==================================" << std::endl;
            //std::cout << "loop time         : " << time << std::endl;
            std::cout << "loop num          : " << loop_count << std::endl;
            std::cout << "solver iteration  : " << solver_iter["iter_count"] << std::endl;
            std::cout << "solver cost       : " << solver_result["f"] << std::endl;
            std::cout << "robot heading(Rad): " << rbt_state_init(2) << std::endl;
            std::cout << "robot velocity    : " << rbt_cmd_msg.twist.linear.x << std::endl;
        }
        
        if (!sub_print_once && (!rbt_odom_in || !rbt_ref_traj_in))
        {
            std::cout << "robot odom && trajectory not subscribed..." << std::endl;
            sub_print_once = true;
        }

        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_ugv");
    mpc_ugv mpc_node;
    mpc_node.node_param_init();
    mpc_node.init();
    mpc_node.run();

    return 0;
}