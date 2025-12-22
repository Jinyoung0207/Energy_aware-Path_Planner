#include "wheelbird_local_planner.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>

namespace wheelbird_planner 
{
    static inline double sqr(double x) { return x*x; }
    
    static inline double getYawFromQuat(const geometry_msgs::Quaternion& q) 
    {
        tf2::Quaternion tq;
        tf2::fromMsg(q, tq);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
        return yaw;
    }
    WheelbirdLocalPlanner::WheelbirdLocalPlanner() : nh_(), pnh_("~")
    {
        loadParams();
        if (use_yaml_anchor_) loadYamlAnchors();
        loadRobots();

        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)), 
                                 &WheelbirdLocalPlanner::onTimer, this);
    }

    void WheelbirdLocalPlanner::loadParams()
    {
        pnh_.param("publish_rate", publish_rate_, 10.0);
        pnh_.param("local_dt", local_dt_, 0.1);
        pnh_.param("horizon_s", horizon_s_, 6.0);
        pnh_.param("goal_hold_s", goal_hold_s_, 2.0);
        pnh_.param("chunk_size", chunk_size_, 30);
        
        pnh_.param<std::string>("global_topic_suffix", global_topic_suffix_, "/planned_path");
        
        pnh_.param("sync_all", sync_all_, false);
        pnh_.param("rebase_if_past", rebase_if_past_, true);
        pnh_.param("log_debug", log_debug_, true);
        pnh_.param("fill_yaw_tangent", fill_yaw_, true);

        pnh_.param("align_start", align_start_, true);
        pnh_.param("align_to_now", align_to_now_, false);
        pnh_.param("start_delay_s", start_delay_s_, 0.5);

        pnh_.param("rebase_to_local", rebase_to_local_, true);
        pnh_.param("apply_rotation", apply_rotation_, true);
        pnh_.param<std::string>("output_frame", output_frame_, "robot_local");
        
        pnh_.param("use_yaml_anchor", use_yaml_anchor_, true);
        pnh_.param("mode_alt_thresh", mode_alt_thresh_, 0.1);
        pnh_.param("mode_window_n", mode_window_n_, 30);
    }

    void WheelbirdLocalPlanner::loadYamlAnchors()
    {
        XmlRpc::XmlRpcValue arr;
        if (nh_.getParam("/energy_aware_path_planner/robots", arr) && arr.getType() == XmlRpc::XmlRpcValue::TypeArray) 
        {
            for (int i=0; i<arr.size(); ++i) 
            {
                auto& r = arr[i];
                if (!r.hasMember("name")) continue;
                std::string name = static_cast<std::string>(r["name"]);
                
                if (r.hasMember("start_xyz")) 
                {
                    auto& s = r["start_xyz"];
                    geometry_msgs::Point p;
                    p.x = (double)s[0]; p.y = (double)s[1]; p.z = (double)s[2];
                    double yaw = 0.0;
                    if (r.hasMember("start_yaw_deg")) yaw = ((double)r["start_yaw_deg"]) * M_PI / 180.0;
                    
                    yaml_anchors_[name] = {p, yaw};
                }
            }
            ROS_INFO("Loaded %zu anchors from YAML.", yaml_anchors_.size());
        } 
        else 
        {
            ROS_WARN("Failed to load robot anchors from param server.");
        }
    }

    void WheelbirdLocalPlanner::loadRobots()
    {
        XmlRpc::XmlRpcValue names;
        if (!pnh_.getParam("robot_names", names)) 
        {
            ROS_ERROR("~robot_names param is missing!");
            return;
        }

        for (int i=0; i<names.size(); ++i) 
        {
            std::string name = static_cast<std::string>(names[i]);
            RobotContext ctx;
            ctx.name = name;
            ctx.nh_ns = ros::NodeHandle(nh_, name);

            // Subscriber: Global Path (Topic: /robot_name/planned_path)
            std::string topic = name + global_topic_suffix_; 
            if (global_topic_suffix_[0] != '/') topic = "/" + name + global_topic_suffix_; 
            
            // [이름 변경] 콜백 클래스명 변경
            ctx.sub_global = nh_.subscribe<nav_msgs::Path>(topic, 1, 
                [this, i](const nav_msgs::Path::ConstPtr& msg){ this->globalPathCb(i, msg); });

            ctx.pub_local = ctx.nh_ns.advertise<nav_msgs::Path>(local_topic_, 1);
            ctx.pub_ref_traj = ctx.nh_ns.advertise<nav_msgs::Path>(ref_traj_topic_, 1);
            ctx.pub_mode = ctx.nh_ns.advertise<std_msgs::Int64>("mode_flag", 1);

            if (yaml_anchors_.count(name)) 
            {
                ctx.anchor = yaml_anchors_[name].first;
                ctx.anchor_yaw = yaml_anchors_[name].second;
                ctx.have_anchor = true;
            }

            robots_.push_back(ctx);
            ROS_INFO("Robot [%s] setup. Listening on: %s", name.c_str(), topic.c_str());
        }
    }

    void WheelbirdLocalPlanner::globalPathCb(size_t idx, const nav_msgs::Path::ConstPtr& msg)
    {
        if (idx >= robots_.size() || !msg || msg->poses.empty()) return;
        auto& ctx = robots_[idx];
        
        ctx.last_global_path = *msg;
        
        auto& P = ctx.last_global_path.poses;
        for (size_t k=1; k<P.size(); ++k) 
        {
            if (P[k].header.stamp < P[k-1].header.stamp) P[k].header.stamp = P[k-1].header.stamp;
        }

        if (!ctx.has_path) 
        {
            ctx.has_path = true;
            ctx.path_start_time = P.front().header.stamp;
            ROS_INFO("[%s] Received first path. Start T=%.2f", ctx.name.c_str(), ctx.path_start_time.toSec());
        }
    }

    void WheelbirdLocalPlanner::tryInitAlignment(const ros::Time& now)
    {
        if (!align_start_ || offsets_initialized_) return;

        if (sync_all_) 
        {
            for (const auto& r : robots_) if (!r.has_path) return;
        } 
        else 
        {
            bool any = false;
            for (const auto& r : robots_) if (r.has_path) any = true;
            if (!any) return;
        }

        if (align_to_now_) 
        {
            base_time_ = now + ros::Duration(start_delay_s_);
        } 
        else 
        {
            ros::Time max_start(0);
            for (const auto& r : robots_) if (r.has_path) max_start = std::max(max_start, r.path_start_time);
            base_time_ = max_start + ros::Duration(start_delay_s_);
        }

        for (auto& r : robots_) 
        {
            if (r.has_path) 
            {
                r.align_offset = base_time_ - r.path_start_time;
                r.has_offset = true;
                if (log_debug_) ROS_INFO("[%s] Alignment Offset: %.2fs", r.name.c_str(), r.align_offset.toSec());
            }
        }
        offsets_initialized_ = true;
    }

    void WheelbirdLocalPlanner::onTimer(const ros::TimerEvent&)
    {
        ros::Time now = ros::Time::now();
        if (sync_all_) 
        {
            for (const auto& r : robots_) if (!r.has_path) return;
        }

        tryInitAlignment(now);

        for (auto& r : robots_) 
        {
            processRobot(r, now);
        }
    }

    void WheelbirdLocalPlanner::processRobot(RobotContext& ctx, const ros::Time& now)
    {
        if (!ctx.has_path) return;
        const auto& path = ctx.last_global_path;
        
        if (rebase_to_local_ && !ctx.have_anchor) 
        {
            ctx.anchor = path.poses.front().pose.position;
            ctx.anchor_yaw = getYawFromQuat(path.poses.front().pose.orientation);
            ctx.have_anchor = true;
            ROS_INFO("[%s] Anchor Auto-Set: (%.2f, %.2f) Yaw: %.2f", 
                     ctx.name.c_str(), ctx.anchor.x, ctx.anchor.y, ctx.anchor_yaw);
        }

        ros::Duration off = (align_start_ && ctx.has_offset) ? ctx.align_offset : ros::Duration(0);
        ros::Time t_first = path.poses.front().header.stamp + off;
        ros::Time t_last  = path.poses.back().header.stamp + ros::Duration(goal_hold_s_) + off;
        
        ros::Time t_win_start = std::max(now, t_first);
        
        if (rebase_if_past_ && t_win_start > t_last) 
        {
            t_win_start = t_last - ros::Duration((chunk_size_-1)*local_dt_);
        }

        if (t_win_start > t_last) return;

        nav_msgs::Path ref_traj; 
        ref_traj.header.frame_id = path.header.frame_id;
        ref_traj.header.stamp = now;
        
        nav_msgs::Path local_path_out;
        local_path_out.header.frame_id = output_frame_;
        local_path_out.header.stamp = now;

        for (int i=0; i<chunk_size_; ++i) 
        {
            ros::Time t_sample = t_win_start + ros::Duration(i * local_dt_);
            ros::Time t_orig = t_sample - off;

            geometry_msgs::Point pos;
            if (!samplePose(path, t_orig, pos)) break; 

            geometry_msgs::PoseStamped ps_ref;
            ps_ref.header = ref_traj.header;
            ps_ref.header.stamp = t_sample;
            ps_ref.pose.position = pos;
            ps_ref.pose.orientation.w = 1.0;

            if (fill_yaw_) 
            {
                double yaw;
                if (computeYaw(path, t_orig, 0.05, yaw)) 
                {
                    tf2::Quaternion q; q.setRPY(0, 0, yaw);
                    ps_ref.pose.orientation = tf2::toMsg(q);
                }
            }
            ref_traj.poses.push_back(ps_ref);

            geometry_msgs::PoseStamped ps_local = ps_ref;
            ps_local.header.frame_id = output_frame_;
            ps_local.header.stamp = now;

            if (rebase_to_local_ && ctx.have_anchor) 
            {
                applyRigidTransform(ps_local, ctx.anchor, ctx.anchor_yaw);
            }
            local_path_out.poses.push_back(ps_local);
        }

        if (!ref_traj.poses.empty()) 
        {
            ctx.pub_ref_traj.publish(ref_traj);
            ctx.pub_local.publish(local_path_out);
            
            int mode = determineMode(ref_traj, ctx.last_mode);
            std_msgs::Int64 m; m.data = mode;
            ctx.pub_mode.publish(m);
            ctx.last_mode = mode;
        }
    }

    bool WheelbirdLocalPlanner::samplePose(const nav_msgs::Path& path, const ros::Time& t, geometry_msgs::Point& out)
    {
        const auto& P = path.poses;
        if (P.empty()) return false;
        if (t <= P.front().header.stamp) { out = P.front().pose.position; return true; }
        if (t >= P.back().header.stamp)  { out = P.back().pose.position;  return true; }

        auto it = std::upper_bound(P.begin(), P.end(), t, 
            [](const ros::Time& val, const geometry_msgs::PoseStamped& p){ return val < p.header.stamp; });
        
        size_t idx = std::distance(P.begin(), it);
        if (idx == 0) idx = 1;
        
        const auto& A = P[idx-1];
        const auto& B = P[idx];
        
        double dt = (B.header.stamp - A.header.stamp).toSec();
        double u = (dt > 1e-9) ? (t - A.header.stamp).toSec() / dt : 0.0;
        
        out.x = A.pose.position.x + (B.pose.position.x - A.pose.position.x) * u;
        out.y = A.pose.position.y + (B.pose.position.y - A.pose.position.y) * u;
        out.z = A.pose.position.z + (B.pose.position.z - A.pose.position.z) * u;
        return true;
    }

    bool WheelbirdLocalPlanner::computeYaw(const nav_msgs::Path& path, const ros::Time& t, double dt, double& out_yaw)
    {
        geometry_msgs::Point p1, p2;
        if (!samplePose(path, t, p1)) return false;
        if (!samplePose(path, t + ros::Duration(dt), p2)) return false;
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        if (std::hypot(dx, dy) < 1e-6) return false;
        
        out_yaw = std::atan2(dy, dx);
        return true;
    }

    void WheelbirdLocalPlanner::applyRigidTransform(geometry_msgs::PoseStamped& ps, const geometry_msgs::Point& anchor, double anchor_yaw)
    {
        double dx = ps.pose.position.x - anchor.x;
        double dy = ps.pose.position.y - anchor.y;
        
        if (apply_rotation_) 
        {
            double c = std::cos(-anchor_yaw);
            double s = std::sin(-anchor_yaw);
            ps.pose.position.x = c*dx - s*dy;
            ps.pose.position.y = s*dx + c*dy;
            
            double yaw = getYawFromQuat(ps.pose.orientation);
            tf2::Quaternion q; q.setRPY(0, 0, yaw - anchor_yaw);
            ps.pose.orientation = tf2::toMsg(q);
        } 
        else
        {
            ps.pose.position.x = dx;
            ps.pose.position.y = dy;
        }
    }

    int WheelbirdLocalPlanner::determineMode(const nav_msgs::Path& local, int last)
    {
        int use_n = std::min((int)local.poses.size(), mode_window_n_);
        if (use_n <= 0) return last;

        bool all_ground = true;
        bool any_air = false;

        for (int i=0; i<use_n; ++i) 
        {
            if (local.poses[i].pose.position.z >= mode_alt_thresh_) 
            {
                all_ground = false;
                any_air = true;
                break;
            }
        }

        if (last == 1) return all_ground ? 0 : 1;
        else           return any_air ? 1 : 0;   
    }

} // namespace wheelbird_planner

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheelbird_local_planner");
    wheelbird_planner::WheelbirdLocalPlanner node;
    ros::spin();
    return 0;
}