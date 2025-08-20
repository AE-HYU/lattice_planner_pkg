#include "local_planner_pkg/utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace local_planner_pkg {

FrenetPoint Utils::cartesian_to_frenet(const Point2D& cartesian_point, 
                                       const std::vector<RefPoint>& reference_path) {
    if (reference_path.empty()) {
        return FrenetPoint();
    }
    
    // Find closest reference point
    int closest_idx = find_closest_reference_index(cartesian_point, reference_path);
    
    if (closest_idx < 0 || closest_idx >= static_cast<int>(reference_path.size())) {
        return FrenetPoint();
    }
    
    const RefPoint& ref_point = reference_path[closest_idx];
    
    // Calculate relative position
    double dx = cartesian_point.x - ref_point.x;
    double dy = cartesian_point.y - ref_point.y;
    
    // Transform to frenet frame
    double cos_theta = std::cos(ref_point.heading);
    double sin_theta = std::sin(ref_point.heading);
    
    FrenetPoint frenet_point;
    frenet_point.s = ref_point.s;
    frenet_point.d = -dx * sin_theta + dy * cos_theta;  // Lateral distance (left positive)
    
    return frenet_point;
}

CartesianPoint Utils::frenet_to_cartesian(const FrenetPoint& frenet_point, 
                                          const std::vector<RefPoint>& reference_path) {
    if (reference_path.empty()) {
        return CartesianPoint();
    }
    
    // Get reference point at s
    RefPoint ref_point = interpolate_reference_point(reference_path, frenet_point.s);
    
    // Transform to cartesian
    double cos_theta = std::cos(ref_point.heading);
    double sin_theta = std::sin(ref_point.heading);
    
    CartesianPoint cartesian_point;
    cartesian_point.x = ref_point.x - frenet_point.d * sin_theta;
    cartesian_point.y = ref_point.y + frenet_point.d * cos_theta;
    cartesian_point.yaw = ref_point.heading;
    cartesian_point.velocity = ref_point.velocity;
    cartesian_point.curvature = ref_point.curvature;
    
    return cartesian_point;
}

std::vector<RefPoint> Utils::load_reference_path_from_csv(const std::string& file_path, double resolution) {
    std::vector<RefPoint> path;
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << file_path << std::endl;
        return path;
    }
    
    std::string line;
    bool first_line = true;
    int x_col = -1, y_col = -1, vel_col = -1;
    
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
        // Parse CSV line
        while (std::getline(ss, cell, ',')) {
            // Trim whitespace
            cell.erase(0, cell.find_first_not_of(" \t"));
            cell.erase(cell.find_last_not_of(" \t") + 1);
            row.push_back(cell);
        }
        
        // Parse header
        if (first_line) {
            for (int i = 0; i < row.size(); ++i) {
                std::string header = row[i];
                std::transform(header.begin(), header.end(), header.begin(), ::tolower);
                
                if (header == "x_m" || header == "x") {
                    x_col = i;
                } else if (header == "y_m" || header == "y") {
                    y_col = i;
                } else if (header == "vx_mps" || header == "v" || header == "velocity") {
                    vel_col = i;
                }
            }
            
            if (x_col == -1 || y_col == -1) {
                std::cerr << "Could not find x and y columns in CSV header" << std::endl;
                return path;
            }
            
            first_line = false;
            continue;
        }
        
        // Parse data rows
        if (row.size() > std::max(x_col, y_col)) {
            RefPoint point;
            try {
                point.x = std::stod(row[x_col]);
                point.y = std::stod(row[y_col]);
                point.velocity = (vel_col >= 0 && vel_col < row.size()) ? 
                                std::stod(row[vel_col]) : 5.0;  // Default velocity
            } catch (const std::exception& e) {
                std::cerr << "Error parsing CSV line: " << line << std::endl;
                continue;
            }
            
            path.push_back(point);
        }
    }
    
    file.close();
    
    if (path.empty()) {
        std::cerr << "No valid points loaded from CSV file" << std::endl;
        return path;
    }
    
    // Calculate derived properties
    calculate_arc_length(path);
    calculate_heading(path);
    calculate_curvature(path);
    
    std::cout << "Loaded " << path.size() << " reference points from " << file_path << std::endl;
    
    return path;
}

void Utils::calculate_arc_length(std::vector<RefPoint>& path) {
    if (path.empty()) return;
    
    path[0].s = 0.0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double ds = std::sqrt(dx*dx + dy*dy);
        path[i].s = path[i-1].s + ds;
    }
}

void Utils::calculate_heading(std::vector<RefPoint>& path) {
    if (path.size() < 2) return;
    
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            // First point: use direction to next point
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            path[i].heading = std::atan2(dy, dx);
        } else if (i == path.size() - 1) {
            // Last point: use direction from previous point
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            path[i].heading = std::atan2(dy, dx);
        } else {
            // Middle points: average of directions
            double dx1 = path[i].x - path[i-1].x;
            double dy1 = path[i].y - path[i-1].y;
            double heading1 = std::atan2(dy1, dx1);
            
            double dx2 = path[i+1].x - path[i].x;
            double dy2 = path[i+1].y - path[i].y;
            double heading2 = std::atan2(dy2, dx2);
            
            // Handle angle wrapping
            double diff = heading2 - heading1;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            
            path[i].heading = normalize_angle(heading1 + diff * 0.5);
        }
    }
}

void Utils::calculate_curvature(std::vector<RefPoint>& path) {
    if (path.size() < 3) return;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        path[i].curvature = calculate_curvature_at_point(path[i-1], path[i], path[i+1]);
    }
    
    // Handle boundary points
    if (path.size() >= 3) {
        path[0].curvature = path[1].curvature;
        path[path.size()-1].curvature = path[path.size()-2].curvature;
    }
}

RefPoint Utils::interpolate_reference_point(const std::vector<RefPoint>& path, double s) {
    if (path.empty()) {
        return RefPoint();
    }
    
    if (s <= path[0].s) {
        return path[0];
    }
    
    if (s >= path.back().s) {
        return path.back();
    }
    
    // Find surrounding points
    size_t i = 0;
    for (i = 0; i < path.size() - 1; ++i) {
        if (path[i + 1].s > s) {
            break;
        }
    }
    
    if (i >= path.size() - 1) {
        return path.back();
    }
    
    // Linear interpolation
    const RefPoint& p1 = path[i];
    const RefPoint& p2 = path[i + 1];
    
    double ratio = (s - p1.s) / (p2.s - p1.s);
    ratio = std::max(0.0, std::min(1.0, ratio));
    
    RefPoint interpolated;
    interpolated.x = p1.x + ratio * (p2.x - p1.x);
    interpolated.y = p1.y + ratio * (p2.y - p1.y);
    interpolated.s = s;
    interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
    interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
    interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
    
    return interpolated;
}

double Utils::calculate_distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

double Utils::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

int Utils::find_closest_reference_index(const Point2D& point, const std::vector<RefPoint>& reference_path) {
    if (reference_path.empty()) {
        return -1;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    int closest_idx = 0;
    
    for (size_t i = 0; i < reference_path.size(); ++i) {
        double distance = calculate_distance(point, Point2D(reference_path[i].x, reference_path[i].y));
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = static_cast<int>(i);
        }
    }
    
    return closest_idx;
}

std::vector<double> Utils::generate_biased_offsets(
    const Point2D& vehicle_pos, const std::vector<RefPoint>& reference_path, 
    const PlannerConfig& config) {
    
    std::vector<double> offsets;
    
    // Find vehicle position relative to reference path
    FrenetPoint vehicle_frenet = cartesian_to_frenet(vehicle_pos, reference_path);
    double current_d = vehicle_frenet.d;
    
    // Get road boundaries at current position
    RefPoint ref_point = interpolate_reference_point(reference_path, vehicle_frenet.s);
    double left_boundary = ref_point.width_left;
    double right_boundary = -ref_point.width_right;
    
    // Determine which side the vehicle is closer to
    double center_offset = (left_boundary + right_boundary) / 2.0;
    double vehicle_offset_from_center = current_d - center_offset;
    
    // Generate biased offsets
    double max_offset = std::min(config.max_lateral_offset, 
                                std::min(left_boundary - 0.2, std::abs(right_boundary) - 0.2));
    
    // Bias towards center - if vehicle is left, sample more on the right (and vice versa)
    double bias_factor = config.bias_strength;
    
    // Generate offsets from left to right (negative to positive)
    // This creates indexed paths from 0 (leftmost) to N-1 (rightmost)
    for (double offset = -max_offset; offset <= max_offset; offset += config.lateral_step) {
        // Calculate distance from vehicle's current lateral position
        double distance_from_vehicle = std::abs(offset - current_d);
        
        // Biased sampling: if vehicle is left of center, favor right offsets
        double bias_weight = 1.0;
        if (vehicle_offset_from_center > 0.1) { // Vehicle is left
            bias_weight = (offset < current_d) ? bias_factor : 1.0;
        } else if (vehicle_offset_from_center < -0.1) { // Vehicle is right
            bias_weight = (offset > current_d) ? bias_factor : 1.0;
        }
        
        // Skip some offsets based on bias (lower bias_weight = higher chance to skip)
        if (bias_weight < bias_factor && (rand() % 100) > (bias_weight * 50)) {
            continue;
        }
        
        offsets.push_back(offset);
    }
    
    return offsets;
}

std::vector<CartesianPoint> Utils::generate_lattice_path(
    const Point2D& start_pos, double start_yaw, double lateral_offset,
    const std::vector<RefPoint>& reference_path, const PlannerConfig& config) {
    
    std::vector<CartesianPoint> path;
    
    // Start from vehicle's current position and heading (not reference path)
    double planning_distance = config.max_velocity * config.planning_horizon;
    int num_points = static_cast<int>(planning_distance / config.path_resolution);
    
    for (int i = 0; i < num_points; ++i) {
        double progress = i * config.path_resolution;
        
        CartesianPoint cart_point;
        
        if (i == 0) {
            // First point starts at vehicle position
            cart_point.x = start_pos.x;
            cart_point.y = start_pos.y;
            cart_point.yaw = start_yaw;
        } else {
            // Transition from vehicle heading to lateral offset relative to reference path
            double transition_factor = std::tanh(progress / 2.0); // Smooth transition over 2 meters
            
            // Find reference point ahead
            Point2D projected_pos;
            projected_pos.x = start_pos.x + progress * std::cos(start_yaw);
            projected_pos.y = start_pos.y + progress * std::sin(start_yaw);
            
            FrenetPoint frenet = cartesian_to_frenet(projected_pos, reference_path);
            
            // Blend from current heading to reference-relative lateral offset
            frenet.d = (1.0 - transition_factor) * 0.0 + transition_factor * lateral_offset;
            
            CartesianPoint ref_point = frenet_to_cartesian(frenet, reference_path);
            cart_point = ref_point;
        }
        
        // Get reference point for speed profile
        RefPoint ref_at_s = interpolate_reference_point(reference_path, 
                                                       cartesian_to_frenet(Point2D(cart_point.x, cart_point.y), reference_path).s);
        
        // Use reference path's own speed profile (it already accounts for curvature)
        cart_point.velocity = ref_at_s.velocity;
        
        cart_point.time = i * config.dt;
        
        path.push_back(cart_point);
    }
    
    return path;
}

double Utils::calculate_path_cost(const PathCandidate& path, 
                                 const std::vector<Obstacle>& obstacles,
                                 const PlannerConfig& config) {
    if (path.points.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    // Refined cost function: cost[i] = lateral_offset + (1 / obstacle_distance)
    double cost = 0.0;
    
    // Lateral offset cost - penalize deviation from centerline
    cost += std::abs(path.lateral_offset);
    
    // Obstacle distance cost - inverse of minimum distance to obstacles
    if (!obstacles.empty()) {
        double min_obstacle_distance = std::numeric_limits<double>::max();
        
        // Find minimum distance from path to any obstacle
        for (const auto& point : path.points) {
            for (const auto& obstacle : obstacles) {
                double distance = calculate_distance(Point2D(point.x, point.y), Point2D(obstacle.x, obstacle.y));
                min_obstacle_distance = std::min(min_obstacle_distance, distance);
            }
        }
        
        // Add inverse distance cost (closer obstacles = higher cost)
        if (min_obstacle_distance > 0.01) { // Avoid division by zero
            cost += (1.0 / min_obstacle_distance);
        } else {
            cost += 1000.0; // Very high cost for extremely close obstacles
        }
    }
    
    return cost;
}

planning_custom_msgs::msg::PathWithVelocity Utils::convert_to_path_with_velocity(const PathCandidate& path) {
    planning_custom_msgs::msg::PathWithVelocity velocity_path;
    
    for (const auto& point : path.points) {
        planning_custom_msgs::msg::PathPoint path_point;
        path_point.x = point.x;
        path_point.y = point.y;
        path_point.yaw = point.yaw;
        path_point.velocity = point.velocity;
        path_point.curvature = point.curvature;
        path_point.time_from_start = point.time;
        
        velocity_path.points.push_back(path_point);
    }
    
    // Set maximum velocity
    double max_vel = 0.0;
    for (const auto& point : path.points) {
        max_vel = std::max(max_vel, point.velocity);
    }
    velocity_path.max_velocity = max_vel;
    
    // Set path ID based on lateral offset
    velocity_path.path_id = static_cast<uint32_t>(std::hash<double>{}(path.lateral_offset) % 10000);
    
    return velocity_path;
}

visualization_msgs::msg::MarkerArray Utils::create_path_markers(
    const std::vector<PathCandidate>& candidates, const PathCandidate& selected) {
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Create markers for candidates
    for (size_t i = 0; i < candidates.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "path_candidates";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;  // Line width
        
        // Color based on safety
        if (candidates[i].is_safe) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.6;
        }
        
        for (const auto& point : candidates[i].points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        markers.markers.push_back(marker);
    }
    
    // Highlight selected path
    if (!selected.points.empty()) {
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.header.frame_id = "map";
        selected_marker.ns = "selected_path";
        selected_marker.id = 0;
        selected_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        selected_marker.action = visualization_msgs::msg::Marker::ADD;
        
        selected_marker.scale.x = 0.1;  // Thicker line
        
        // Blue for selected path
        selected_marker.color.r = 0.0;
        selected_marker.color.g = 0.0;
        selected_marker.color.b = 1.0;
        selected_marker.color.a = 1.0;
        
        for (const auto& point : selected.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.1;  // Slightly elevated
            selected_marker.points.push_back(p);
        }
        
        markers.markers.push_back(selected_marker);
    }
    
    return markers;
}

double Utils::calculate_curvature_at_point(const RefPoint& prev, const RefPoint& curr, const RefPoint& next) {
    // Use finite differences to approximate curvature
    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double ds1 = std::sqrt(dx1*dx1 + dy1*dy1);
    double ds2 = std::sqrt(dx2*dx2 + dy2*dy2);
    
    if (ds1 < 1e-6 || ds2 < 1e-6) {
        return 0.0;
    }
    
    // Approximate curvature
    double curvature = 2.0 * cross_product / (ds1 * ds2 * (ds1 + ds2));
    
    return curvature;
}

} // namespace local_planner_pkg