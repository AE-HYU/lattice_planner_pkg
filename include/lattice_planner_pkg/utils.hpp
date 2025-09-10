#pragma once

#include "lattice_planner.hpp"
#include <vector>
#include <string>

namespace lattice_planner_pkg {

class Utils {
public:
    // Coordinate transformations
    static FrenetPoint cartesian_to_frenet(const Point2D& cartesian_point, 
                                           const std::vector<RefPoint>& reference_path);
    static CartesianPoint frenet_to_cartesian(const FrenetPoint& frenet_point, 
                                              const std::vector<RefPoint>& reference_path);
    
    // Path utilities
    static std::vector<RefPoint> load_reference_path_from_csv(const std::string& file_path, 
                                                             double resolution = 0.1);
    static void calculate_arc_length(std::vector<RefPoint>& path);
    static void calculate_heading(std::vector<RefPoint>& path);
    static void calculate_curvature(std::vector<RefPoint>& path);
    static void close_raceline_loop(std::vector<RefPoint>& path);
    static RefPoint interpolate_reference_point(const std::vector<RefPoint>& path, double s);
    
    // Geometry utilities
    static double calculate_distance(const Point2D& p1, const Point2D& p2);
    static double normalize_angle(double angle);
    static int find_closest_reference_index(const Point2D& point, 
                                           const std::vector<RefPoint>& reference_path);
    
    // Path generation
    static std::vector<CartesianPoint> generate_lattice_path(
        const Point2D& start_pos, double start_yaw, double lateral_offset,
        const std::vector<RefPoint>& reference_path, const PlannerConfig& config);
    
    // Biased sampling for path candidates
    static std::vector<double> generate_biased_offsets(
        const Point2D& vehicle_pos, const std::vector<RefPoint>& reference_path, 
        const PlannerConfig& config);
    
    // Speed control with sigmoid function
    static double calculate_obstacle_speed_factor(const std::vector<CartesianPoint>& path_points,
                                                 const std::vector<Obstacle>& obstacles,
                                                 const PlannerConfig& config);
    
    
    // Cost calculation
    static double calculate_path_cost(const PathCandidate& path, 
                                     const std::vector<Obstacle>& obstacles,
                                     const std::vector<RefPoint>& reference_path,
                                     const PlannerConfig& config);
    
    // Conversion utilities
    static crazy_planner_msgs::msg::WaypointArray convert_to_waypoint_array(const PathCandidate& path, const std::vector<RefPoint>& reference_path = std::vector<RefPoint>());
    static visualization_msgs::msg::MarkerArray create_path_markers(
        const std::vector<PathCandidate>& candidates, const PathCandidate& selected);

private:
    static double calculate_curvature_at_point(const RefPoint& prev, const RefPoint& curr, const RefPoint& next);
};

} // namespace lattice_planner_pkg