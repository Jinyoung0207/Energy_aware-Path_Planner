#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <XmlRpcValue.h>

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <optional>

namespace wheelbird_planner 
{

struct RobotContext 
{
    std::string name;
    ros::NodeHandle nh_ns;
    ros::Subscriber sub_global;
    ros::Publisher  pub_local;
    ros::Publisher  pub_ref_traj;
    ros::Publisher  pub_mode;

    nav_msgs::Path  last_global_path;
    bool      has_path{false};
    ros::Time path_start_time;

    // Alignment
    bool          has_offset{false};
    ros::Duration align_offset{0.0};

    // Anchor (Local Frame Origin)
    bool                 have_anchor{false};
    geometry_msgs::Point anchor;
    double               anchor_yaw{0.0};
    
    // Mode State
    int last_mode{1}; // 0: Ground, 1: Air
};

// [이름 변경] MultiLocalPathBuilder -> WheelbirdLocalPlanner
class WheelbirdLocalPlanner 
{
public:
    WheelbirdLocalPlanner();
    ~WheelbirdLocalPlanner() = default;

private:
    // --- Callbacks ---
    void onTimer(const ros::TimerEvent& event);
    void globalPathCb(size_t robot_idx, const nav_msgs::Path::ConstPtr& msg);

    // --- Helpers ---
    void loadParams();
    void loadRobots();
    void loadYamlAnchors();
    
    void tryInitAlignment(const ros::Time& now);
    void processRobot(RobotContext& ctx, const ros::Time& now);
    
    // Math Helpers
    bool samplePose(const nav_msgs::Path& path, const ros::Time& t, geometry_msgs::Point& out_pos);
    bool computeYaw(const nav_msgs::Path& path, const ros::Time& t, double dt, double& out_yaw);
    void applyRigidTransform(geometry_msgs::PoseStamped& pose, const geometry_msgs::Point& anchor, double anchor_yaw);
    int  determineMode(const nav_msgs::Path& local_path, int last_mode);

    // --- Members ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer timer_;

    std::vector<RobotContext> robots_;
    std::map<std::string, std::pair<geometry_msgs::Point, double>> yaml_anchors_;

    // Params
    double publish_rate_{10.0};
    double local_dt_{0.1};
    double horizon_s_{6.0};
    double goal_hold_s_{2.0};
    int    chunk_size_{30};
    
    bool   sync_all_{false};
    bool   rebase_if_past_{true};
    bool   log_debug_{true};
    bool   fill_yaw_{true};

    // Alignment Params
    bool   align_start_{true};
    bool   align_to_now_{false};
    double start_delay_s_{0.5};
    bool   offsets_initialized_{false};
    ros::Time base_time_;

    // Transformation Params
    bool   rebase_to_local_{true};
    bool   apply_rotation_{true};
    bool   use_yaml_anchor_{true};
    std::string output_frame_{"robot_local"};

    // Mode Params
    double mode_alt_thresh_{0.1};
    int    mode_window_n_{30};
    
    // Topics
    std::string global_topic_suffix_{"/planned_path"}; 
    std::string local_topic_{"local_path"};
    std::string ref_traj_topic_{"ref_trajectory"};
};

} // namespace wheelbird_planner