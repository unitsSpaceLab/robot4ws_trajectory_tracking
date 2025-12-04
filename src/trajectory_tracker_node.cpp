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

<<<<<<< HEAD
//=============================================================================
// SIMPLE PID
//=============================================================================
=======


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
>>>>>>> 47ba6c3... updating
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

<<<<<<< HEAD
//=============================================================================
// HELPERS 
//=============================================================================
=======
// HELPERS 
>>>>>>> 47ba6c3... updating
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

<<<<<<< HEAD
//=============================================================================
// MAIN NODE
//=============================================================================
=======
// MAIN NODE
>>>>>>> 47ba6c3... updating
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

<<<<<<< HEAD
=======
        // PID gains
>>>>>>> 47ba6c3... updating
        pnh.param<double>("kp_dx", kp_dx_, 1.0);
        pnh.param<double>("kd_dx", kd_dx_, 0.0);
        pnh.param<double>("ki_dx", ki_dx_, 0.0);
        pnh.param<double>("kp_dy", kp_dy_, 1.0);
        pnh.param<double>("kd_dy", kd_dy_, 0.0);
        pnh.param<double>("ki_dy", ki_dy_, 0.0);
        pnh.param<double>("kp_dyaw", kp_dyaw_, 0.07);
        pnh.param<double>("kd_dyaw", kd_dyaw_, 0.05);
        pnh.param<double>("ki_dyaw", ki_dyaw_, 0.0);

        // Initialize PIDs
        pid_dx_ = std::make_unique<PID>(kp_dx_, kd_dx_, ki_dx_);
        pid_dy_ = std::make_unique<PID>(kp_dy_, kd_dy_, ki_dy_);
        pid_dyaw_ = std::make_unique<PID>(kp_dyaw_, kd_dyaw_, ki_dyaw_);

        // ROS interface
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/Archimede/cmd_vel", 1);
        link_states_sub_ = nh.subscribe("/gazebo/link_states", 1, &TrajectoryTracker::linkStatesCallback, this);

        // Call kinematic mode service
        setKinematicMode(kinematic_mode_);

        // Load CSV if provided
        if (!csv_path_.empty()) {
            loadCSV(csv_path_);
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
<<<<<<< HEAD
        std::getline(file, line); // skip header
=======
        std::getline(file, line);
>>>>>>> 47ba6c3... updating

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string id, x_str, y_str;
            std::getline(ss, id, ',');
            std::getline(ss, x_str, ',');
            std::getline(ss, y_str, ',');
            raw.emplace_back(std::stod(x_str), std::stod(y_str));
        }

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


            ROS_INFO("Pose: (%.2f, %.2f) Yaw: %.2f | Lookahead: (%.2f, %.2f) | errX: %.2f errY: %.2f",
                pose_.x, pose_.y, yaw_, lookaheadPt.x, lookaheadPt.y, errX, errY);


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

            ROS_INFO_THROTTLE(1.0, "Seg: %d/%zu | Curv: %.3f | Speed: %.2f | YawErr: %.2f",
                current_segment_, waypoints_.size(), curvature, velNorm * coeff, orientationError);

            rate.sleep();
        }
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
