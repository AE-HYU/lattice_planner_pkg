#include "local_planner_pkg/local_planner.hpp"
#include "local_planner_pkg/utils.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>

namespace local_planner_pkg {

LocalPlanner::LocalPlanner() 
    : Node("local_planner"),
      vehicle_yaw_(0.0),
      vehicle_velocity_(0.0),
      odom_received_(false) {
    
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
    this->declare_parameter("planning_frequency", config_.planning_frequency);
    
    config_.reference_path_file = this->get_parameter("reference_path_file").as_string();
    config_.path_resolution = this->get_parameter("path_resolution").as_double();
    config_.lateral_step = this->get_parameter("lateral_step").as_double();
    config_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
    config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
    config_.max_velocity = this->get_parameter("max_velocity").as_double();
    config_.vehicle_width = this->get_parameter("vehicle_width").as_double();
    config_.planning_frequency = this->get_parameter("planning_frequency").as_double();
    
    // Load reference path
    if (!load_reference_path()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference path");
        return false;
    }
    
    // Initialize publishers - only PathWithVelocity and visualization
    path_with_velocity_pub_ = this->create_publisher<planning_custom_msgs::msg::PathWithVelocity>(
        "/planned_path_with_velocity", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_candidates", 10);
    
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
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("local_planner_pkg");
            resolved_path = pkg_share_dir + "/config/reference_paths/" + config_.reference_path_file + ".csv";
        }
        
        reference_path_ = Utils::load_reference_path_from_csv(resolved_path, config_.path_resolution);
    }
    
    if (reference_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Reference path is empty");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Reference path loaded with %zu points", reference_path_.size());
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
    
    // Select best path
    PathCandidate selected_path = select_best_path(candidates);
    
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
        
        // Check if path stays within road boundaries
        bool within_boundaries = true;
        for (const auto& point : candidate.points) {
            if (!Utils::is_within_road_boundaries(point, reference_path_)) {
                within_boundaries = false;
                break;
            }
        }
        
        if (!within_boundaries) {
            candidate.out_of_track = true;
            continue;
        }
        
        // Check safety using occupancy grid
        candidate.is_safe = is_path_safe_in_grid(candidate);
        
        // Apply speed reduction only if this path is avoiding obstacles (not following raceline)
        if (!obstacles.empty() && std::abs(candidate.lateral_offset) > 0.1) {
            apply_speed_reduction_for_avoidance(candidate, obstacles);
        }
        
        // Calculate cost
        candidate.cost = Utils::calculate_path_cost(candidate, obstacles, config_);
        
        candidates.push_back(candidate);
    }
    
    return candidates;
}

PathCandidate LocalPlanner::select_best_path(const std::vector<PathCandidate>& candidates) {
    if (candidates.empty()) {
        return PathCandidate();
    }
    
    // Step 1: Check for collision across all paths
    bool any_collision = false;
    int reference_path_index = -1;
    
    // Find reference path (lateral_offset closest to 0) and check for collisions
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (!candidates[i].is_safe) {
            any_collision = true;
        }
        
        // Find reference path (closest to zero lateral offset)
        if (reference_path_index == -1 || 
            std::abs(candidates[i].lateral_offset) < std::abs(candidates[reference_path_index].lateral_offset)) {
            reference_path_index = static_cast<int>(i);
        }
    }
    
    // Step 2: If no collision detected, select reference path
    if (!any_collision && reference_path_index >= 0) {
        RCLCPP_DEBUG(this->get_logger(), "No collision detected - selecting reference path (offset: %.2f)", 
                     candidates[reference_path_index].lateral_offset);
        return candidates[reference_path_index];
    }
    
    // Step 3: If collision detected, find path with minimum cost
    RCLCPP_DEBUG(this->get_logger(), "Collision detected - calculating costs for optimal path");
    
    double min_cost = std::numeric_limits<double>::max();
    size_t best_index = 0;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].cost < min_cost) {
            min_cost = candidates[i].cost;
            best_index = i;
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Selected path with offset: %.2f, cost: %.3f", 
                 candidates[best_index].lateral_offset, min_cost);
    
    return candidates[best_index];
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

bool LocalPlanner::is_path_safe_in_grid(const PathCandidate& path) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    if (!current_grid_) {
        return true; // No grid data, assume safe
    }
    
    // Use occupancy grid to check path safety
    return Utils::is_path_clear_in_grid(path.points, current_grid_, 
                                       config_.vehicle_width, 
                                       50); // 50% occupancy threshold
}

bool LocalPlanner::check_collision(const CartesianPoint& point, const std::vector<Obstacle>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = Utils::calculate_distance(Point2D(point.x, point.y), Point2D(obstacle.x, obstacle.y));
        if (distance < config_.vehicle_width) {
            return true;
        }
    }
    return false;
}

void LocalPlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) {
        return;
    }
    
    // Publish only PathWithVelocity - contains all necessary information
    auto velocity_path = Utils::convert_to_path_with_velocity(path);
    velocity_path.header.stamp = this->get_clock()->now();
    velocity_path.header.frame_id = "map";
    path_with_velocity_pub_->publish(velocity_path);
}

void LocalPlanner::publish_visualization(const std::vector<PathCandidate>& candidates, 
                                        const PathCandidate& selected) {
    auto markers = Utils::create_path_markers(candidates, selected);
    marker_pub_->publish(markers);
}

} // namespace local_planner_pkg

// Main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<local_planner_pkg::LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}