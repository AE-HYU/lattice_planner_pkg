#include "lattice_planner_pkg/lattice_planner.hpp"
#include "lattice_planner_pkg/utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>

namespace lattice_planner_pkg {

LocalPlanner::LocalPlanner() 
    : Node("lattice_planner"),
      vehicle_yaw_(0.0),
      vehicle_velocity_(0.0),
      odom_received_(false),
      has_previous_path_(false) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize local planner");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Local Planner initialized successfully");
}

LocalPlanner::~LocalPlanner() {}

bool LocalPlanner::initialize() {
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Declare and get parameters
    this->declare_parameter("reference_path_file", config_.reference_path_file);
    this->declare_parameter("path_resolution", config_.path_resolution);
    this->declare_parameter("lateral_step", config_.lateral_step);
    this->declare_parameter("max_lateral_offset", config_.max_lateral_offset);
    this->declare_parameter("planning_horizon", config_.planning_horizon);
    this->declare_parameter("max_velocity", config_.max_velocity);
    this->declare_parameter("vehicle_width", config_.vehicle_width);
    this->declare_parameter("safety_margin", config_.safety_margin);
    this->declare_parameter("planning_frequency", config_.planning_frequency);
    this->declare_parameter("min_planning_distance", config_.min_planning_distance);
    this->declare_parameter("transition_smoothness", config_.transition_smoothness);
    this->declare_parameter("curvature_weight", config_.curvature_weight);
    this->declare_parameter("wheelbase", config_.wheelbase);
    this->declare_parameter("front_lookahead", config_.front_lookahead);
    this->declare_parameter("hazard_detection_distance", config_.hazard_detection_distance);
    this->declare_parameter("min_hazard_distance", config_.min_hazard_distance);
    this->declare_parameter("min_speed_ratio", config_.min_speed_ratio);
    this->declare_parameter("speed_sigmoid_steepness", config_.speed_sigmoid_steepness);
    
    config_.reference_path_file = this->get_parameter("reference_path_file").as_string();
    config_.path_resolution = this->get_parameter("path_resolution").as_double();
    config_.lateral_step = this->get_parameter("lateral_step").as_double();
    config_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
    config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
    config_.max_velocity = this->get_parameter("max_velocity").as_double();
    config_.vehicle_width = this->get_parameter("vehicle_width").as_double();
    config_.safety_margin = this->get_parameter("safety_margin").as_double();
    config_.planning_frequency = this->get_parameter("planning_frequency").as_double();
    config_.min_planning_distance = this->get_parameter("min_planning_distance").as_double();
    config_.transition_smoothness = this->get_parameter("transition_smoothness").as_double();
    config_.curvature_weight = this->get_parameter("curvature_weight").as_double();
    config_.wheelbase = this->get_parameter("wheelbase").as_double();
    config_.front_lookahead = this->get_parameter("front_lookahead").as_double();
    config_.hazard_detection_distance = this->get_parameter("hazard_detection_distance").as_double();
    config_.min_hazard_distance = this->get_parameter("min_hazard_distance").as_double();
    config_.min_speed_ratio = this->get_parameter("min_speed_ratio").as_double();
    config_.speed_sigmoid_steepness = this->get_parameter("speed_sigmoid_steepness").as_double();
    
    // Debug: Print loaded parameters
    RCLCPP_INFO(this->get_logger(), "Loaded parameters:");
    RCLCPP_INFO(this->get_logger(), "  min_planning_distance: %.2f", config_.min_planning_distance);
    RCLCPP_INFO(this->get_logger(), "  transition_smoothness: %.2f", config_.transition_smoothness);
    RCLCPP_INFO(this->get_logger(), "  curvature_weight: %.2f", config_.curvature_weight);
    RCLCPP_INFO(this->get_logger(), "  planning_horizon: %.2f", config_.planning_horizon);
    
    // Load reference path
    if (!load_reference_path()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference path");
        return false;
    }
    
    // Initialize publishers - PathWithVelocity, visualization, and reference path
    waypoint_array_pub_ = this->create_publisher<ae_hyu_msgs::msg::WpntArray>(
        "/local_waypoints", 10);
    global_waypoint_array_pub_ = this->create_publisher<ae_hyu_msgs::msg::WpntArray>(
        "/global_waypoints", 10);
    frenet_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_candidates", 10);
    reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/reference_path", 10);
    
    // Initialize subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LocalPlanner::odom_callback, this, std::placeholders::_1));
    
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&LocalPlanner::grid_callback, this, std::placeholders::_1));
    
    obstacles_sub_ = this->create_subscription<obstacle_detection_pkg::msg::ObstacleArray>(
        "/detected_obstacles", 10, std::bind(&LocalPlanner::obstacles_callback, this, std::placeholders::_1));
    
    // Initialize planning timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / config_.planning_frequency));
    planning_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LocalPlanner::planning_timer_callback, this));
    
    return true;
}

bool LocalPlanner::load_reference_path() {
    if (config_.reference_path_file.empty()) {
        RCLCPP_WARN(this->get_logger(), "No reference path file specified, creating test path");
        
        // Create a simple test path
        reference_path_.clear();
        for (int i = 0; i < 100; ++i) {
            RefPoint point;
            point.x = i * 0.5;
            point.y = 2.0 * std::sin(i * 0.1);
            point.velocity = config_.max_velocity;
            reference_path_.push_back(point);
        }
        
        Utils::calculate_arc_length(reference_path_);
        Utils::calculate_heading(reference_path_);
        Utils::calculate_curvature(reference_path_);
    } else {
        // Load from CSV file
        std::string resolved_path;
        if (config_.reference_path_file.find('/') != std::string::npos) {
            resolved_path = config_.reference_path_file;
        } else {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("lattice_planner_pkg");
            resolved_path = pkg_share_dir + "/config/reference_paths/" + config_.reference_path_file + ".csv";
        }
        
        reference_path_ = Utils::load_reference_path_from_csv(resolved_path, config_.path_resolution);
    }
    
    if (reference_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Reference path is empty");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Reference path loaded with %zu points from file: %s", reference_path_.size(), config_.reference_path_file.c_str());
    RCLCPP_INFO(this->get_logger(), "First point: (%.3f, %.3f), Last point: (%.3f, %.3f)", 
                reference_path_.front().x, reference_path_.front().y,
                reference_path_.back().x, reference_path_.back().y);
    return true;
}

void LocalPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    
    vehicle_position_.x = msg->pose.pose.position.x;
    vehicle_position_.y = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicle_yaw_ = yaw;
    
    // Calculate velocity
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_velocity_ = std::sqrt(vx*vx + vy*vy);
    
    current_odom_ = *msg;  // Store full odometry for frenet conversion
    odom_received_ = true;
}

void LocalPlanner::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    // Store the updated occupancy grid - it already contains obstacles
    current_grid_ = msg;
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
        "Received updated occupancy grid: %dx%d, resolution=%.3f", 
        msg->info.width, msg->info.height, msg->info.resolution);
}

void LocalPlanner::obstacles_callback(const obstacle_detection_pkg::msg::ObstacleArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    current_obstacles_.clear();
    for (const auto& obs_msg : msg->obstacles) {
        Obstacle obs;
        obs.x = obs_msg.position.x;           // Global coordinates
        obs.y = obs_msg.position.y;
        obs.z = obs_msg.position.z;
        obs.size = obs_msg.size;              // Approximate size/radius
        obs.distance = obs_msg.distance;      // Distance from robot
        obs.point_count = obs_msg.point_count; // Number of points in cluster
        current_obstacles_.push_back(obs);
    }
    
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Received %zu obstacles from detection", current_obstacles_.size());
}

void LocalPlanner::planning_timer_callback() {
    if (!odom_received_) {
        return;
    }
    
    // Publish reference path for visualization (once per second)
    static auto last_ref_path_publish = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_ref_path_publish).count() > 1000) {
        publish_reference_path();
        last_ref_path_publish = now;
    }
    
    // Publish global waypoints and frenet odometry (similar to planner_pkg)
    publish_global_waypoints();
    publish_frenet_odometry();
    
    plan_paths();
}

void LocalPlanner::plan_paths() {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Generate path candidates
    std::vector<PathCandidate> candidates = generate_path_candidates();
    
    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid path candidates generated");
        return;
    }
    
    // Select best path with smooth transition consideration
    PathCandidate selected_path = select_best_path(candidates);
    
    // Apply smooth transition logic
    selected_path = apply_smooth_transition(selected_path, candidates);
    
    // Update path history
    {
        std::lock_guard<std::mutex> lock(path_history_mutex_);
        last_selected_path_ = selected_path;
        has_previous_path_ = true;
    }
    
    // Publish results
    publish_selected_path(selected_path);
    publish_visualization(candidates, selected_path);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    RCLCPP_DEBUG(this->get_logger(), "Planning completed in %ld ms with %zu candidates", 
                 duration.count(), candidates.size());
}

std::vector<PathCandidate> LocalPlanner::generate_path_candidates() {
    std::vector<PathCandidate> candidates;
    
    Point2D vehicle_pos;
    double vehicle_yaw;
    std::vector<Obstacle> obstacles;
    
    {
        std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
        vehicle_yaw = vehicle_yaw_;
    }
    
    {
        std::lock_guard<std::mutex> lock(obstacles_mutex_);
        obstacles = current_obstacles_;
    }
    
    // Generate biased lateral offsets based on vehicle position
    std::vector<double> biased_offsets = Utils::generate_biased_offsets(vehicle_pos, reference_path_, config_);
    
    // Generate path candidates for each biased offset
    for (double offset : biased_offsets) {
        PathCandidate candidate;
        candidate.lateral_offset = offset;
        candidate.points = Utils::generate_lattice_path(vehicle_pos, vehicle_yaw, offset, 
                                                       reference_path_, config_);
        
        if (candidate.points.empty()) {
            continue;
        }
        
        // Apply sigmoid-based obstacle speed reduction
        double speed_factor = Utils::calculate_obstacle_speed_factor(candidate.points, obstacles, config_);
        for (auto& point : candidate.points) {
            point.velocity *= speed_factor;
        }
        
        // Check if path stays within track boundaries using reference path width data
        bool stays_within_track = check_track_boundaries(candidate);
        candidate.out_of_track = !stays_within_track;
        
        if (candidate.out_of_track) {
            candidate.is_safe = false;
            RCLCPP_DEBUG(this->get_logger(), "Path rejected: out of track boundaries (offset: %.2f)", 
                        candidate.lateral_offset);
            continue;
        }
        
        // Check safety using both occupancy grid and detected obstacles
        candidate.is_safe = is_path_safe_in_grid(candidate) && is_path_safe(candidate);
        
        // Speed reduction will be handled during path cost calculation
        
        // Calculate cost
        candidate.cost = Utils::calculate_path_cost(candidate, obstacles, reference_path_, config_);
        
        candidates.push_back(candidate);
    }
    
    return candidates;
}

PathCandidate LocalPlanner::select_best_path(const std::vector<PathCandidate>& candidates) {
    if (candidates.empty()) {
        return PathCandidate();
    }
    
    // Step 1: Store all generated candidate paths in container (already done via candidates parameter)
    // paths[i] represents each potential trajectory
    
    // Step 2: Collision Check - Iterate through all paths from i = 0 to paths.size() - 1
    bool collision_detected = false;
    int reference_path_index = -1;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        // Perform collision check for each path
        if (!candidates[i].is_safe) {
            collision_detected = true;
        }
        
        // Find reference path (lateral_offset closest to 0.0)
        if (reference_path_index == -1 || 
            std::abs(candidates[i].lateral_offset) < std::abs(candidates[reference_path_index].lateral_offset)) {
            reference_path_index = static_cast<int>(i);
        }
    }
    
    // Step 3: If no collision detected on any path, return predefined reference path
    if (!collision_detected && reference_path_index >= 0) {
        RCLCPP_DEBUG(this->get_logger(), "No collision detected - returning reference path (offset: %.2f)", 
                     candidates[reference_path_index].lateral_offset);
        return candidates[reference_path_index];
    }
    
    // Step 4: Cost Calculation (if collision detected)
    // For each path paths[i], calculate cost = lateral_offset + (1 / obstacle_distance)
    RCLCPP_DEBUG(this->get_logger(), "Collision detected - proceeding with cost-based selection");
    
    // Step 5: Optimal Path Selection using std::min_element for efficiency
    auto min_cost_it = std::min_element(candidates.begin(), candidates.end(),
        [](const PathCandidate& a, const PathCandidate& b) {
            return a.cost < b.cost;
        });
    
    if (min_cost_it != candidates.end()) {
        size_t best_index = std::distance(candidates.begin(), min_cost_it);
        RCLCPP_DEBUG(this->get_logger(), "Selected optimal path [%zu] with offset: %.2f, cost: %.3f", 
                     best_index, min_cost_it->lateral_offset, min_cost_it->cost);
        
        // Step 6: Return the optimal path to next module (control unit)
        return *min_cost_it;
    }
    
    // Fallback: return first candidate if min_element fails
    return candidates[0];
}

PathCandidate LocalPlanner::apply_smooth_transition(const PathCandidate& selected_path, 
                                                   const std::vector<PathCandidate>& candidates) {
    std::lock_guard<std::mutex> lock(path_history_mutex_);
    
    if (!has_previous_path_) {
        return selected_path; // No previous path, use selected
    }
    
    // Check if the new path is drastically different from the previous one
    double offset_change = std::abs(selected_path.lateral_offset - last_selected_path_.lateral_offset);
    double transition_threshold = config_.lateral_step * 2.0; // Allow max 2 steps change
    
    if (offset_change <= transition_threshold) {
        return selected_path; // Smooth transition, use selected path
    }
    
    // Large change detected - find intermediate path for smoother transition
    RCLCPP_DEBUG(this->get_logger(), "Large offset change detected: %.2f -> %.2f (change: %.2f)", 
                 last_selected_path_.lateral_offset, selected_path.lateral_offset, offset_change);
    
    // Find path closer to previous selection for smoother transition
    double target_offset = last_selected_path_.lateral_offset;
    if (selected_path.lateral_offset > last_selected_path_.lateral_offset) {
        target_offset += transition_threshold; // Move gradually right
    } else {
        target_offset -= transition_threshold; // Move gradually left
    }
    
    // Find candidate closest to target offset
    PathCandidate smooth_transition = selected_path;
    double min_diff = std::abs(selected_path.lateral_offset - target_offset);
    
    for (const auto& candidate : candidates) {
        if (candidate.is_safe && !candidate.out_of_track) {
            double diff = std::abs(candidate.lateral_offset - target_offset);
            if (diff < min_diff) {
                min_diff = diff;
                smooth_transition = candidate;
            }
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Smooth transition: using offset %.2f instead of %.2f", 
                 smooth_transition.lateral_offset, selected_path.lateral_offset);
    
    return smooth_transition;
}

bool LocalPlanner::is_path_safe(const PathCandidate& path) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    for (const auto& point : path.points) {
        if (check_collision(point, current_obstacles_)) {
            return false;
        }
    }
    
    return true;
}

bool LocalPlanner::check_track_boundaries(const PathCandidate& path) {
    // Check if path stays within track boundaries using reference path width data
    for (const auto& point : path.points) {
        // Convert path point to Frenet coordinates to get lateral offset
        FrenetPoint frenet = Utils::cartesian_to_frenet(Point2D(point.x, point.y), reference_path_);
        
        // Get reference point at this location to check boundaries
        RefPoint ref_point = Utils::interpolate_reference_point(reference_path_, frenet.s);
        
        // Check if lateral offset exceeds track boundaries
        if (frenet.d > ref_point.width_left || frenet.d < -ref_point.width_right) {
            RCLCPP_DEBUG(this->get_logger(), "Point (%.2f, %.2f) outside track: d=%.2f, left_limit=%.2f, right_limit=%.2f", 
                        point.x, point.y, frenet.d, ref_point.width_left, -ref_point.width_right);
            return false;
        }
    }
    
    return true;
}

bool LocalPlanner::is_path_safe_in_grid(const PathCandidate& path) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    if (!current_grid_) {
        return true; // No grid data, assume safe
    }
    
    // Check each point in the path against the occupancy grid
    for (const auto& point : path.points) {
        // Convert world coordinates to grid coordinates
        int grid_x = static_cast<int>((point.x - current_grid_->info.origin.position.x) / current_grid_->info.resolution);
        int grid_y = static_cast<int>((point.y - current_grid_->info.origin.position.y) / current_grid_->info.resolution);
        
        // Check bounds
        if (grid_x < 0 || grid_x >= static_cast<int>(current_grid_->info.width) ||
            grid_y < 0 || grid_y >= static_cast<int>(current_grid_->info.height)) {
            continue; // Point outside grid, skip
        }
        
        // Check occupancy value
        int index = grid_y * current_grid_->info.width + grid_x;
        if (index >= 0 && index < static_cast<int>(current_grid_->data.size())) {
            int8_t occupancy_value = current_grid_->data[index];
            
            // Consider cells with occupancy > 50 as occupied (walls/obstacles)
            if (occupancy_value > 50) {
                RCLCPP_DEBUG(this->get_logger(), "Path point (%.2f, %.2f) hits occupied cell: occupancy=%d", 
                            point.x, point.y, occupancy_value);
                return false;
            }
        }
    }
    
    return true;
}

bool LocalPlanner::check_collision(const CartesianPoint& point, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = Utils::calculate_distance(Point2D(point.x, point.y), Point2D(obstacle.x, obstacle.y));
        // Use obstacle size + vehicle width for collision detection
        double collision_threshold = obstacle.size + config_.vehicle_width;
        if (distance < collision_threshold) {
            return true;
        }
    }
    return false;
}

void LocalPlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) {
        return;
    }
    
    // Publish WaypointArray - contains all necessary information
    auto waypoint_array = Utils::convert_to_waypoint_array(path, reference_path_);
    waypoint_array.header.stamp = this->get_clock()->now();
    waypoint_array.header.frame_id = "map";
    waypoint_array_pub_->publish(waypoint_array);
}

void LocalPlanner::publish_visualization(const std::vector<PathCandidate>& candidates, 
                                        const PathCandidate& selected) {
    auto markers = Utils::create_path_markers(candidates, selected);
    marker_pub_->publish(markers);
}

void LocalPlanner::publish_reference_path() {
    if (reference_path_.empty()) {
        return;
    }
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "map";
    
    // Convert reference path to nav_msgs::Path
    for (const auto& ref_point : reference_path_) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = ref_point.x;
        pose_stamped.pose.position.y = ref_point.y;
        pose_stamped.pose.position.z = 0.0;
        
        // Simple quaternion from heading (avoid TF2 operations that might cause segfault)
        double heading = ref_point.heading;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = std::sin(heading / 2.0);
        pose_stamped.pose.orientation.w = std::cos(heading / 2.0);
        
        path_msg.poses.push_back(pose_stamped);
    }
    
    
    reference_path_pub_->publish(path_msg);
    RCLCPP_INFO(this->get_logger(), "Published reference path with %zu points", path_msg.poses.size());
}

void LocalPlanner::publish_global_waypoints() {
    if (reference_path_.empty()) {
        return;
    }
    
    ae_hyu_msgs::msg::WpntArray waypoint_array;
    waypoint_array.header.stamp = this->now();
    waypoint_array.header.frame_id = "map";
    
    for (size_t i = 0; i < reference_path_.size(); ++i) {
        const auto& ref_point = reference_path_[i];
        
        ae_hyu_msgs::msg::Wpnt waypoint;
        waypoint.id = static_cast<int32_t>(i);
        waypoint.s_m = ref_point.s;
        waypoint.d_m = 0.0;
        waypoint.x_m = ref_point.x;
        waypoint.y_m = ref_point.y;
        waypoint.d_right = ref_point.width_right;
        waypoint.d_left = ref_point.width_left;
        waypoint.psi_rad = ref_point.heading;
        waypoint.kappa_radpm = ref_point.curvature;
        waypoint.vx_mps = ref_point.velocity;
        waypoint.ax_mps2 = 0.0;  // Not available in lattice planner
        
        waypoint_array.wpnts.push_back(waypoint);
    }
    
    global_waypoint_array_pub_->publish(waypoint_array);
}

void LocalPlanner::publish_frenet_odometry() {
    if (!odom_received_ || reference_path_.empty()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    
    // Use optimized Utils function for frenet conversion
    FrenetPoint frenet_point = Utils::cartesian_to_frenet_optimized(
        Point2D(vehicle_position_.x, vehicle_position_.y), reference_path_, last_closest_index_);
    
    // Convert velocities to frenet coordinates using Utils
    double vx = current_odom_.twist.twist.linear.x;
    double vy = current_odom_.twist.twist.linear.y;
    auto frenet_vel = Utils::cartesian_velocity_to_frenet(vx, vy, vehicle_yaw_, reference_path_, last_closest_index_);
    double vs = frenet_vel.first;   // longitudinal velocity
    double vd = frenet_vel.second;  // lateral velocity
    
    // Create frenet odometry message
    nav_msgs::msg::Odometry frenet_odom;
    frenet_odom.header.stamp = current_odom_.header.stamp;
    frenet_odom.header.frame_id = "frenet";
    frenet_odom.child_frame_id = "base_link_frenet";
    
    // Pose: s, d coordinates
    frenet_odom.pose.pose.position.x = frenet_point.s;  // s coordinate
    frenet_odom.pose.pose.position.y = frenet_point.d;  // d coordinate
    frenet_odom.pose.pose.position.z = 0.0;
    
    // Orientation (identity quaternion for frenet frame)
    frenet_odom.pose.pose.orientation.x = 0.0;
    frenet_odom.pose.pose.orientation.y = 0.0;
    frenet_odom.pose.pose.orientation.z = 0.0;
    frenet_odom.pose.pose.orientation.w = 1.0;
    
    // Twist: vs, vd velocities
    frenet_odom.twist.twist.linear.x = vs;  // longitudinal velocity
    frenet_odom.twist.twist.linear.y = vd;  // lateral velocity
    frenet_odom.twist.twist.linear.z = 0.0;
    frenet_odom.twist.twist.angular.x = 0.0;
    frenet_odom.twist.twist.angular.y = 0.0;
    frenet_odom.twist.twist.angular.z = 0.0;
    
    // Covariances (set to zero)
    for (int i = 0; i < 36; ++i) {
        frenet_odom.pose.covariance[i] = 0.0;
        frenet_odom.twist.covariance[i] = 0.0;
    }
    
    frenet_odom_pub_->publish(frenet_odom);
}

// Removed: Duplicate frenet conversion functions now handled by Utils class

} // namespace lattice_planner_pkg

// Main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lattice_planner_pkg::LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}