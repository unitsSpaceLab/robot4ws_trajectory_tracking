#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <robot4ws_msgs/kinematic_mode.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <gazebo_msgs/SpawnModel.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>

#include <set>

#include <thread>



// Pure Pursuit Algorithm:
//
// 1. Densify the waypoint path for smooth curvature and uniform spacing.
//
// 2. At each control step:
//    a) Project the robot onto the path → get closest point (current reference).
//    b) Compute local curvature using the Menger formula : κ = (4 × Area) / (|AB| × |BC| × |CA|).
//    c) Adjust lookahead distance and max speed inversely to curvature
//       (tighter curves → smaller lookahead, slower speed).
//
// 3. Compute the pursuit target:
//    - Draw a circle centered on the robot with radius = lookahead distance.
//    - Find the intersection of this circle with the path ahead.
//    - Use that intersection as the target point.
//
// 4. Control the robot:
//    - Transform target point into robot's local frame.
//    - Apply PID on position error (x, y) and heading error to compute velocity commands.
//
// 5. Loop until the goal is reached.


// !! the min and max lookahead must be chosen carefully + PID yaw must be reduced in case of rough terrain. 

// SIMPLE PID
struct PID {
    double kp, kd, ki;
    double integral = 0.0;
    double prev_error = 0.0;
    bool first = true;
    

    PID(double p, double d, double i) : kp(p), kd(d), ki(i) {}

    double run(double error, double dt) {
        integral += error * dt;
        double derivative = first ? 0.0 : (error - prev_error) / dt;
        prev_error = error;
        first = false;
        return kp * error + kd * derivative + ki * integral;
    }

    void resetIntegral() { integral = 0.0; }
};

// HELPERS 
struct Point2D {
    double x, y;
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
    Point2D operator-(const Point2D& o) const { return {x - o.x, y - o.y}; }
    Point2D operator+(const Point2D& o) const { return {x + o.x, y + o.y}; }
    Point2D operator*(double s) const { return {x * s, y * s}; }
    double norm() const { return std::sqrt(x*x + y*y); }
    double dot(const Point2D& o) const { return x*o.x + y*o.y; }
};

Point2D projectPointOnSegment(const Point2D& pt, const Point2D& p1, const Point2D& p2, double& t) {
    Point2D v = p2 - p1;
    Point2D u = pt - p1;
    double lenSq = v.dot(v);
    if (lenSq < 1e-10) { t = 0; return p1; }
    t = std::clamp(u.dot(v) / lenSq, 0.0, 1.0);
    return p1 + v * t;
}

void findClosestPointOnPath(const Point2D& robotPos, const std::vector<Point2D>& waypoints,
                            Point2D& closestPoint, int& segmentIdx, double& param) {
    double minDist = 1e9;
    segmentIdx = 0;
    param = 0;
    closestPoint = waypoints[0];

    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        double t;
        Point2D pt = projectPointOnSegment(robotPos, waypoints[i], waypoints[i+1], t);
        double dist = (robotPos - pt).norm();
        if (dist < minDist) {
            minDist = dist;
            closestPoint = pt;
            segmentIdx = i;
            param = t;
        }
    }
}

bool circleSegmentIntersection(const Point2D& center, double radius,
                               const Point2D& p1, const Point2D& p2,
                               Point2D& pt1, Point2D& pt2, bool& hasPt1, bool& hasPt2) {
    Point2D d = p2 - p1;
    Point2D f = p1 - center;
    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - radius * radius;
    double disc = b*b - 4*a*c;

    hasPt1 = hasPt2 = false;
    if (disc < 0) return false;

    double sqrtDisc = std::sqrt(disc);
    double t1 = (-b - sqrtDisc) / (2*a);
    double t2 = (-b + sqrtDisc) / (2*a);

    if (t1 >= 0 && t1 <= 1) { pt1 = p1 + d * t1; hasPt1 = true; }
    if (t2 >= 0 && t2 <= 1) { pt2 = p1 + d * t2; hasPt2 = true; }
    return hasPt1 || hasPt2;
}

Point2D findLookaheadPoint(const Point2D& robotPos, const std::vector<Point2D>& waypoints,
                           int startIdx, double lookaheadDist) {
    for (size_t i = startIdx; i < waypoints.size() - 1; i++) {
        Point2D pt1, pt2;
        bool has1, has2;
        if (circleSegmentIntersection(robotPos, lookaheadDist, waypoints[i], waypoints[i+1], pt1, pt2, has1, has2)) {
            if (has1 && has2) {
                return ((pt2 - waypoints[i]).norm() > (pt1 - waypoints[i]).norm()) ? pt2 : pt1;
            }
            return has2 ? pt2 : pt1;
        }
    }
    return waypoints[std::min((size_t)startIdx + 1, waypoints.size() - 1)];
}

double computeCurvature(const std::vector<Point2D>& wp, int idx) {
    if (idx < 1 || idx >= (int)wp.size() - 1) return 0;
    Point2D p1 = wp[idx-1], p2 = wp[idx], p3 = wp[idx+1];
    double a = (p2 - p1).norm();
    double b = (p3 - p2).norm();
    double c = (p3 - p1).norm();
    double area = std::abs((p2.x-p1.x)*(p3.y-p1.y) - (p3.x-p1.x)*(p2.y-p1.y)) / 2.0;
    double denom = a * b * c;
    return (denom < 1e-10) ? 0 : (4.0 * area / denom);
}

std::vector<Point2D> densifyPath(const std::vector<Point2D>& wp, double maxSpacing) {
    std::vector<Point2D> result;
    for (size_t i = 0; i < wp.size() - 1; i++) {
        double dist = (wp[i+1] - wp[i]).norm();
        int n = std::max(1, (int)std::ceil(dist / maxSpacing));
        for (int j = 0; j < n; j++) {
            double t = (double)j / n;
            result.push_back(wp[i] + (wp[i+1] - wp[i]) * t);
        }
    }
    result.push_back(wp.back());
    return result;
}

std::vector<double> computeAllCurvatures(const std::vector<Point2D>& wp) {
    std::vector<double> curv(wp.size(), 0);
    for (size_t i = 1; i < wp.size() - 1; i++) curv[i] = computeCurvature(wp, i);
    if (wp.size() >= 2) { curv[0] = curv[1]; curv.back() = curv[curv.size()-2]; }
    return curv;
}

// MAIN NODE
class TrajectoryTracker {
public:
    TrajectoryTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh) {
        // Load parameters
        pnh.param<std::string>("csv_path", csv_path_, "");
        pnh.param<std::string>("tracked_link", tracked_link_, "Archimede::Archimede_base_link");
        pnh.param<std::string>("kinematic_mode", kinematic_mode_, "symmetric_ackermann");
        pnh.param<double>("max_speed", max_speed_, 0.5);
        pnh.param<double>("max_orientation_velocity", max_orientation_velocity_, 0.5);
        pnh.param<double>("max_acceleration", max_acceleration_, 0.5);
        pnh.param<double>("min_lookahead_distance", min_lookahead_distance_, 0.25);
        pnh.param<double>("max_lookahead_distance", max_lookahead_distance_, 1.5);
        pnh.param<double>("curvature_slowdown", curvature_slowdown_, 20.0);
        pnh.param<double>("max_waypoint_spacing", max_waypoint_spacing_, 0.2);
        pnh.param<double>("target_tolerance", target_tolerance_, 0.1);

        // PID gains
        pnh.param<double>("kp_dx", kp_dx_, 1.0);
        pnh.param<double>("kd_dx", kd_dx_, 0.0);
        pnh.param<double>("ki_dx", ki_dx_, 0.0);
        pnh.param<double>("kp_dy", kp_dy_, 1.0);
        pnh.param<double>("kd_dy", kd_dy_, 0.0);
        pnh.param<double>("ki_dy", ki_dy_, 0.0);
        pnh.param<double>("kp_dyaw", kp_dyaw_, 0.07);
        pnh.param<double>("kd_dyaw", kd_dyaw_, 0.05);
        pnh.param<double>("ki_dyaw", ki_dyaw_, 0.0);

        // show waypoints on gazebo
        pnh.param<bool>("show_waypoints", show_waypoints_, true);


        // show markers on rviz
        pnh.param<bool>("show_rviz", show_rviz_, true);

        // Initialize PIDs
        pid_dx_ = std::make_unique<PID>(kp_dx_, kd_dx_, ki_dx_);
        pid_dy_ = std::make_unique<PID>(kp_dy_, kd_dy_, ki_dy_);
        pid_dyaw_ = std::make_unique<PID>(kp_dyaw_, kd_dyaw_, ki_dyaw_);

        // ROS interface
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/Archimede/cmd_vel", 1);
        link_states_sub_ = nh.subscribe("/gazebo/link_states", 1, &TrajectoryTracker::linkStatesCallback, this);
        if (show_rviz_) {
            marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_markers", 1);
        }

        // Call kinematic mode service
        setKinematicMode(kinematic_mode_);

        // Load CSV if provided
        if (!csv_path_.empty()) {
            loadCSV(csv_path_);
            //if you need to show all the nominal trajecotry (all waypoints once) on gazebo just activate this if (show_waypoints_) drawPathInGazebo();
        }

        ROS_INFO("[TrajectoryTracker] Initialized. Waypoints: %zu", waypoints_.size());
    }

    void setKinematicMode(const std::string& mode) {
        ros::ServiceClient client = nh_.serviceClient<robot4ws_msgs::kinematic_mode>("/kinematic_mode");
        robot4ws_msgs::kinematic_mode srv;
        srv.request.requested_kinematic_mode = mode;

        if (client.waitForExistence(ros::Duration(3.0))) {
            if (client.call(srv)) {
                ROS_INFO("[TrajectoryTracker] Set kinematic mode: %s", mode.c_str());
            } else {
                ROS_WARN("[TrajectoryTracker] Failed to call kinematic_mode service");
            }
        } else {
            ROS_WARN("[TrajectoryTracker] kinematic_mode service not available");
        }
    }

    void loadCSV(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open CSV: %s", path.c_str());
            return;
        }

        std::vector<Point2D> raw;
        std::string line;
        std::getline(file, line);

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string id, x_str, y_str;
            std::getline(ss, id, ',');
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            raw.emplace_back(std::stod(x_str), std::stod(y_str));
        }
        waypoints_original_ = raw;
        waypoints_ = densifyPath(raw, max_waypoint_spacing_);
        curvatures_ = computeAllCurvatures(waypoints_);
        ROS_INFO("Loaded %zu waypoints, densified to %zu", raw.size(), waypoints_.size());
    }

    void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
        auto it = std::find(msg->name.begin(), msg->name.end(), tracked_link_);
        if (it == msg->name.end()) return;

        size_t idx = std::distance(msg->name.begin(), it);
        pose_.x = msg->pose[idx].position.x;
        pose_.y = msg->pose[idx].position.y;
        pose_z_ = msg->pose[idx].position.z;

        tf2::Quaternion q(
            msg->pose[idx].orientation.x,
            msg->pose[idx].orientation.y,
            msg->pose[idx].orientation.z,
            msg->pose[idx].orientation.w
        );
        tf2::Matrix3x3 m(q);
        double r, p;
        m.getRPY(r, p, yaw_);
        state_received_ = true;
    }

    void spin() {
        ros::Rate rate(50);
        ros::Time last_time = ros::Time::now();

        while (ros::ok()) {
            ros::spinOnce();

            if (!state_received_ || waypoints_.empty()) {
                rate.sleep();
                continue;
            }

            ros::Time now = ros::Time::now();
            double dt = (now - last_time).toSec();
            last_time = now;
            if (dt <= 0 || dt > 1.0) dt = 0.02;

            // Find closest point
            Point2D closest;
            int closestIdx;
            double closestParam;
            findClosestPointOnPath(pose_, waypoints_, closest, closestIdx, closestParam);
            current_segment_ = std::max(current_segment_, closestIdx);

            if (show_waypoints_ && current_segment_ != last_drawn_segment_) {
                std::thread(&TrajectoryTracker::updateGazeboMarkers, this, pose_z_).detach();
                last_drawn_segment_ = current_segment_;
            }

            // Interpolate curvature
            double curv1 = curvatures_[closestIdx];
            double curv2 = curvatures_[std::min(closestIdx + 1, (int)curvatures_.size() - 1)];
            double curvature = curv1 + closestParam * (curv2 - curv1);

            // Adaptive lookahead
            double curvFactor = 1.0 / (1.0 + 5.0 * curvature);
            double lookahead = min_lookahead_distance_ + 
                (max_lookahead_distance_ - min_lookahead_distance_) * curvFactor;

            // Find lookahead point
            Point2D lookaheadPt = findLookaheadPoint(pose_, waypoints_, current_segment_, lookahead);

            // publish markers
            if (show_rviz_) publishMarkers(lookaheadPt, closest);

            // Target orientation
            Point2D toLookahead = lookaheadPt - pose_;
            double targetYaw = std::atan2(toLookahead.y, toLookahead.x);
            double orientationError = std::atan2(std::sin(targetYaw - yaw_), std::cos(targetYaw - yaw_));

            // Transform to local frame
            double cosYaw = std::cos(yaw_), sinYaw = std::sin(yaw_);
            double errX = cosYaw * toLookahead.x + sinYaw * toLookahead.y;
            double errY = -sinYaw * toLookahead.x + cosYaw * toLookahead.y;

            // PID control
            double velX = pid_dx_->run(errX, dt);
            double velY = pid_dy_->run(errY, dt);
            double velYaw = pid_dyaw_->run(orientationError, dt);


            // ROS_INFO("Pose: (%.2f, %.2f) Yaw: %.2f | Lookahead: (%.2f, %.2f) | errX: %.2f errY: %.2f",
            //     pose_.x, pose_.y, yaw_, lookaheadPt.x, lookaheadPt.y, errX, errY);


            // Curvature-based speed limit
            double speedFactor = std::max(0.2, 1.0 / (1.0 + curvature_slowdown_ * curvature));
            double maxSpeed = std::min(max_speed_ * speedFactor, 1.0);

            // Scale velocity
            double velNorm = std::sqrt(velX*velX + velY*velY);
            double minSpeed = maxSpeed * 0.2;
            double coeff = 1.0;
            if (velNorm < minSpeed && velNorm > 1e-6) coeff = minSpeed / velNorm;
            else if (velNorm > maxSpeed) coeff = maxSpeed / velNorm;
            velX *= coeff;
            velY *= coeff;

            // Clamp orientation velocity
            if (std::abs(velYaw) > max_orientation_velocity_) {
                velYaw = std::copysign(max_orientation_velocity_, velYaw);
            }

            // Apply acceleration limit
            double accelX = (velX - prev_velX_) / dt;
            double accelY = (velY - prev_velY_) / dt;
            double accelNorm = std::sqrt(accelX*accelX + accelY*accelY);
            if (accelNorm > max_acceleration_) {
                double scale = max_acceleration_ / accelNorm;
                velX = prev_velX_ + accelX * scale * dt;
                velY = prev_velY_ + accelY * scale * dt;
            }
            prev_velX_ = velX;
            prev_velY_ = velY;

            // Check progress
            if (closestIdx > current_segment_ || 
                (closestIdx == current_segment_ && closestParam > 0.9)) {
                current_segment_ = closestIdx;
                pid_dyaw_->resetIntegral();
            }

            // Check completion
            double distToGoal = (waypoints_.back() - pose_).norm();
            if (current_segment_ >= (int)waypoints_.size() - 2 && distToGoal < target_tolerance_) {
                ROS_INFO("[TrajectoryTracker] Complete!");
                geometry_msgs::Twist stop;
                cmd_vel_pub_.publish(stop);
                break;
            }

            // Publish
            geometry_msgs::Twist cmd;
            cmd.linear.x = velX;
            cmd.linear.y = velY;
            cmd.angular.z = velYaw;
            cmd_vel_pub_.publish(cmd);

            // ROS_INFO_THROTTLE(1.0, "Seg: %d/%zu | Curv: %.3f | Speed: %.2f | YawErr: %.2f",
            //     current_segment_, waypoints_.size(), curvature, velNorm * coeff, orientationError);

            rate.sleep();
        }
    }


    void updateGazeboMarkers(double robot_z) {
        ros::ServiceClient spawner = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        
        double z = robot_z + 0.5;
        
        // Map to original index
        int orig_idx = current_segment_ * (int)waypoints_original_.size() / (int)waypoints_.size();
        orig_idx = std::min(orig_idx, (int)waypoints_original_.size() - 1);
        
        if (spawned_markers_.find(orig_idx) == spawned_markers_.end()) {
            std::string sdf = R"(
    <?xml version="1.0"?>
    <sdf version="1.5">
    <model name="wp_)" + std::to_string(orig_idx) + R"(">
        <static>true</static>
        <link name="link">
        <visual name="v">
            <pose>)" + std::to_string(waypoints_original_[orig_idx].x) + " " + 
                    std::to_string(waypoints_original_[orig_idx].y) + " " + 
                    std::to_string(z) + R"( 0 0 0</pose>
            <geometry><sphere><radius>0.25</radius></sphere></geometry>
            <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
        </link>
    </model>
    </sdf>)";
            
            gazebo_msgs::SpawnModel srv;
            srv.request.model_name = "wp_" + std::to_string(orig_idx);
            srv.request.model_xml = sdf;
            srv.request.reference_frame = "";
            
            if (spawner.call(srv) && srv.response.success) {
                spawned_markers_.insert(orig_idx);
            }
        }
    }


     // to draw all points!
    void drawPathInGazebo() {
        ros::ServiceClient spawner = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        spawner.waitForExistence(ros::Duration(2.0));
        
        std::stringstream visuals;
        int step = std::max(1, (int)waypoints_.size() / 100);
        
        for (size_t i = 0; i < waypoints_.size(); i += step) {
            visuals << R"(
            <visual name="wp_)" << i << R"(">
            <pose>)" << waypoints_[i].x << " " << waypoints_[i].y << R"( 10.0 0 0 0</pose>
            <geometry><sphere><radius>0.5</radius></sphere></geometry>
            <material><ambient>0 1 1 1</ambient><diffuse>0 1 1 1</diffuse></material>
            </visual>)";
        }
        
        std::string sdf = R"(
    <?xml version="1.0"?>
    <sdf version="1.5">
    <model name="path_markers">
        <static>true</static>
        <link name="link">)" + visuals.str() + R"(
        </link>
    </model>
    </sdf>)";
        
        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = "path_markers";
        srv.request.model_xml = sdf;
        srv.request.reference_frame = "";
        
        if (spawner.call(srv) && srv.response.success)
            ROS_INFO("Path markers spawned");
        else
            ROS_ERROR("Spawn failed: %s", srv.response.status_message.c_str());
    }


    void publishMarkers(const Point2D& lookaheadPt, const Point2D& closestPt) {
        visualization_msgs::MarkerArray markers;
        
        // Path (blue line)
        visualization_msgs::Marker path;
        path.header.frame_id = "Archimede_foot_start";
        path.header.stamp = ros::Time::now();
        path.ns = "path"; path.id = 0;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.pose.orientation.w = 1.0;
        path.scale.x = 0.05;
        path.color.b = 1.0; path.color.a = 1.0;
        for (auto& wp : waypoints_) {
            geometry_msgs::Point p; p.x = wp.x; p.y = wp.y; p.z = 0;
            path.points.push_back(p);
        }
        markers.markers.push_back(path);
        
        // Robot trajectory (green line)
        trajectory_history_.push_back(pose_);
        visualization_msgs::Marker traj = path;
        traj.ns = "trajectory"; traj.id = 1;
        traj.color.b = 0; traj.color.g = 1.0;
        traj.points.clear();
        for (auto& pt : trajectory_history_) {
            geometry_msgs::Point p; p.x = pt.x; p.y = pt.y; p.z = 0;
            traj.points.push_back(p);
        }
        markers.markers.push_back(traj);
        
        // Lookahead point (magenta sphere)
        visualization_msgs::Marker la;
        la.header = path.header;
        la.ns = "lookahead"; la.id = 2;
        la.type = visualization_msgs::Marker::SPHERE;
        la.pose.position.x = lookaheadPt.x;
        la.pose.position.y = lookaheadPt.y;
        la.pose.orientation.w = 1.0;
        la.scale.x = la.scale.y = la.scale.z = 0.15;
        la.color.r = 1.0; la.color.b = 1.0; la.color.a = 1.0;
        markers.markers.push_back(la);
        
        // Closest point (cyan sphere)
        visualization_msgs::Marker cp = la;
        cp.ns = "closest"; cp.id = 3;
        cp.pose.position.x = closestPt.x;
        cp.pose.position.y = closestPt.y;
        cp.color.r = 0; cp.color.g = 1.0;
        markers.markers.push_back(cp);
        
        marker_pub_.publish(markers);
    }   

private:
    ros::NodeHandle nh_;
    std::string csv_path_, tracked_link_, kinematic_mode_;
    double max_speed_, max_orientation_velocity_, max_acceleration_;
    double min_lookahead_distance_, max_lookahead_distance_;
    double curvature_slowdown_, max_waypoint_spacing_, target_tolerance_;
    double kp_dx_, kd_dx_, ki_dx_, kp_dy_, kd_dy_, ki_dy_, kp_dyaw_, kd_dyaw_, ki_dyaw_;

    Point2D pose_;
    double yaw_ = 0;
    bool state_received_ = false;
    int current_segment_ = 0;
    double prev_velX_ = 0, prev_velY_ = 0;
    bool show_waypoints_;
    bool show_rviz_;
    double pose_z_ = 0;
    std::set<int> spawned_markers_;
    int last_drawn_segment_ = -1;
    std::vector<Point2D> waypoints_original_;


    ros::Publisher marker_pub_;
    std::vector<Point2D> trajectory_history_;

    std::vector<Point2D> waypoints_;
    std::vector<double> curvatures_;

    std::unique_ptr<PID> pid_dx_, pid_dy_, pid_dyaw_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber link_states_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_tracker_node");
    ros::NodeHandle nh, pnh("~");
    TrajectoryTracker tracker(nh, pnh);
    tracker.spin();
    return 0;
}
