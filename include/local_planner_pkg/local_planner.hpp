#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <planning_custom_msgs/msg/path_with_velocity.hpp>
#include <obstacle_detection_pkg/msg/obstacle_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <memory>
#include <mutex>
#include <string>

namespace local_planner_pkg {

struct Point2D {
    double x, y;
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct FrenetPoint {
    double s, d;        // longitudinal, lateral position
    double s_dot, d_dot; // velocities
    FrenetPoint() : s(0.0), d(0.0), s_dot(0.0), d_dot(0.0) {}
};

struct CartesianPoint {
    double x, y, yaw, velocity, curvature, time;
    CartesianPoint() : x(0.0), y(0.0), yaw(0.0), velocity(0.0), curvature(0.0), time(0.0) {}
};

struct RefPoint {
    double x, y, s, heading, curvature, velocity;
    double width_left, width_right;  // Road boundaries
    RefPoint() : x(0.0), y(0.0), s(0.0), heading(0.0), curvature(0.0), velocity(5.0), 
                 width_left(2.0), width_right(2.0) {}
};

struct Obstacle {
    double x, y, z;           // Global coordinates from obstacle_detection_pkg
    double size;              // Approximate size/radius of obstacle
    double distance;          // Distance from robot to obstacle centroid
    int32_t point_count;      // Number of points in this obstacle cluster
    Obstacle() : x(0.0), y(0.0), z(0.0), size(0.5), distance(0.0), point_count(0) {}
};

struct PathCandidate {
    std::vector<CartesianPoint> points;
    double cost, lateral_offset;
    bool is_safe, out_of_track;
    PathCandidate() : cost(0.0), lateral_offset(0.0), is_safe(true), out_of_track(false) {}
};

struct PlannerConfig {
    std::string reference_path_file;
    double path_resolution = 0.1;
    double lateral_step = 0.3;
    double max_lateral_offset = 1.5;
    double planning_horizon = 3.0;          // Increased for better lookahead
    double dt = 0.1;
    double max_velocity = 5.0;
    double max_curvature = 1.0;
    double vehicle_width = 0.35;        // Vehicle width for collision checking
    double planning_frequency = 20.0;
    
    // Path smoothing parameters (to fix lattice deformation)
    double min_planning_distance = 15.0;    // Minimum planning distance (m)
    double transition_smoothness = 5.0;     // Transition smoothness factor
    double curvature_weight = 0.5;          // Weight for curvature continuity
    
    // Vehicle geometry for front-wheel planning
    double wheelbase = 0.32;                // Distance from rear to front axle (m)
    double front_lookahead = 0.5;           // Additional lookahead beyond front axle (m)
    
    // Biased sampling parameters
    double bias_strength = 2.0;        // How much to bias towards center
    double min_sampling_density = 0.1; // Minimum offset step size
    double max_sampling_density = 0.5; // Maximum offset step size
    
    // Speed control parameters (for obstacle avoidance)
    double hazard_detection_distance = 3.0;
    double min_hazard_distance = 0.5;
    double min_speed_ratio = 0.1;      // Minimum speed as fraction of reference speed
};

class LocalPlanner : public rclcpp::Node {
public:
    LocalPlanner();
    ~LocalPlanner();

private:
    // Core functionality
    bool initialize();
    bool load_reference_path();
    void plan_paths();
    
    // Path generation and selection
    std::vector<PathCandidate> generate_path_candidates();
    PathCandidate select_best_path(const std::vector<PathCandidate>& candidates);
    PathCandidate apply_smooth_transition(const PathCandidate& selected_path, 
                                         const std::vector<PathCandidate>& candidates);
    
    // Safety checks
    bool is_path_safe(const PathCandidate& path);
    bool is_path_safe_in_grid(const PathCandidate& path);
    bool check_track_boundaries(const PathCandidate& path);
    bool check_collision(const CartesianPoint& point, const std::vector<Obstacle>& obstacles);
    
    
    // Publishing
    void publish_selected_path(const PathCandidate& path);
    void publish_visualization(const std::vector<PathCandidate>& candidates, const PathCandidate& selected);
    void publish_reference_path();
    
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void obstacles_callback(const obstacle_detection_pkg::msg::ObstacleArray::SharedPtr msg);
    void planning_timer_callback();
    
    // Configuration and state
    PlannerConfig config_;
    std::vector<RefPoint> reference_path_;
    
    // Vehicle state
    Point2D vehicle_position_;
    double vehicle_yaw_, vehicle_velocity_;
    bool odom_received_;
    std::mutex vehicle_state_mutex_;
    
    // Path history for smooth transitions
    PathCandidate last_selected_path_;
    bool has_previous_path_;
    std::mutex path_history_mutex_;
    
    // Occupancy grid and obstacles
    nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
    std::vector<Obstacle> current_obstacles_;  // From detection pkg (supplementary)
    std::mutex obstacles_mutex_;
    
    // ROS components
    rclcpp::Publisher<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<obstacle_detection_pkg::msg::ObstacleArray>::SharedPtr obstacles_sub_;
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

} // namespace local_planner_pkg