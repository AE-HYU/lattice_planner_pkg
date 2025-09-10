#include "lattice_planner_pkg/utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lattice_planner_pkg {

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
    int x_col = -1, y_col = -1, vel_col = -1, d_right_col = -1, d_left_col = -1;
    
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        
        // Check if this is a header comment line (contains column names)
        if (line[0] == '#' && first_line) {
            // Check if this comment contains column headers (s_m, x_m, y_m, etc.)
            if (line.find("x_m") != std::string::npos && line.find("y_m") != std::string::npos) {
                // Remove the '#' and treat as header
                line = line.substr(1);
            } else {
                continue;  // Skip other comment lines
            }
        } else if (line[0] == '#') {
            continue;  // Skip comment lines after header is found
        }
        
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
        // Parse CSV line (semicolon-separated)
        while (std::getline(ss, cell, ';')) {
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
                } else if (header == "d_right") {
                    d_right_col = i;
                } else if (header == "d_left") {
                    d_left_col = i;
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
                point.width_right = (d_right_col >= 0 && d_right_col < row.size()) ? 
                                   std::stod(row[d_right_col]) : 2.0;  // Default width
                point.width_left = (d_left_col >= 0 && d_left_col < row.size()) ? 
                                  std::stod(row[d_left_col]) : 2.0;   // Default width
                
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
    
    // Note: Loop closure is handled during interpolation, not by adding physical points
    
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

void Utils::close_raceline_loop(std::vector<RefPoint>& path) {
    if (path.size() < 3) return;
    
    RefPoint& first_point = path.front();
    RefPoint& last_point = path.back();
    
    // Calculate distance between start and end points
    double dx = first_point.x - last_point.x;
    double dy = first_point.y - last_point.y;
    double gap_distance = std::sqrt(dx*dx + dy*dy);
    
    std::cout << "Raceline gap distance: " << gap_distance << " meters" << std::endl;
    
    // If gap is significant (> 0.5m), add intermediate points to close the loop
    if (gap_distance > 0.5) {
        std::cout << "Closing raceline loop with intermediate points..." << std::endl;
        
        // Number of points to add based on gap distance
        int num_interpolation_points = static_cast<int>(gap_distance / 0.2); // Every 20cm
        num_interpolation_points = std::max(1, std::min(num_interpolation_points, 10)); // Limit 1-10 points
        
        double total_length_before = last_point.s;
        
        for (int i = 1; i <= num_interpolation_points; ++i) {
            double ratio = static_cast<double>(i) / (num_interpolation_points + 1);
            
            RefPoint interpolated;
            interpolated.x = last_point.x + ratio * (first_point.x - last_point.x);
            interpolated.y = last_point.y + ratio * (first_point.y - last_point.y);
            interpolated.velocity = (last_point.velocity + first_point.velocity) / 2.0;
            interpolated.width_left = (last_point.width_left + first_point.width_left) / 2.0;
            interpolated.width_right = (last_point.width_right + first_point.width_right) / 2.0;
            
            // Calculate arc length for the new point
            double prev_x = (i == 1) ? last_point.x : path.back().x;
            double prev_y = (i == 1) ? last_point.y : path.back().y;
            double prev_s = (i == 1) ? last_point.s : path.back().s;
            
            double segment_length = std::sqrt((interpolated.x - prev_x)*(interpolated.x - prev_x) + 
                                            (interpolated.y - prev_y)*(interpolated.y - prev_y));
            interpolated.s = prev_s + segment_length;
            
            // Calculate heading direction towards next point
            double next_x = (i == num_interpolation_points) ? first_point.x : 
                           last_point.x + (ratio + 1.0/(num_interpolation_points + 1)) * (first_point.x - last_point.x);
            double next_y = (i == num_interpolation_points) ? first_point.y : 
                           last_point.y + (ratio + 1.0/(num_interpolation_points + 1)) * (first_point.y - last_point.y);
            
            interpolated.heading = std::atan2(next_y - interpolated.y, next_x - interpolated.x);
            interpolated.curvature = 0.0; // Will be recalculated later
            
            path.push_back(interpolated);
        }
        
        std::cout << "Added " << num_interpolation_points << " points to close the loop" << std::endl;
    }
    
    // Update the total track length and ensure smooth connection
    if (!path.empty()) {
        // Add final segment length from last added point back to start
        RefPoint& new_last = path.back();
        double final_dx = first_point.x - new_last.x;
        double final_dy = first_point.y - new_last.y;
        double final_segment = std::sqrt(final_dx*final_dx + final_dy*final_dy);
        
        // Store total track length for wraparound calculations
        double total_track_length = new_last.s + final_segment;
        
        // Update heading for the last few points to point toward start
        if (path.size() >= 2) {
            new_last.heading = std::atan2(first_point.y - new_last.y, first_point.x - new_last.x);
        }
    }
}

RefPoint Utils::interpolate_reference_point(const std::vector<RefPoint>& path, double s) {
    if (path.empty()) {
        return RefPoint();
    }
    
    double total_length = path.back().s;
    
    // Calculate gap distance for closed loop
    double dx = path[0].x - path.back().x;
    double dy = path[0].y - path.back().y;
    double gap_distance = std::sqrt(dx*dx + dy*dy);
    double extended_length = total_length + gap_distance;
    
    // Handle wraparound for closed-loop racelines
    while (s > extended_length) {
        s -= extended_length;
    }
    while (s < 0.0) {
        s += extended_length;
    }
    
    if (s <= path[0].s) {
        return path[0];
    }
    
    if (s >= path.back().s) {
        // Wraparound: interpolate between last and first points
        const RefPoint& p1 = path.back();
        const RefPoint& p2 = path.front();
        
        // Calculate physical gap distance between last and first point
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double gap_distance = std::sqrt(dx*dx + dy*dy);
        
        double remaining_s = s - p1.s;
        
        if (gap_distance > 1e-6) {
            double ratio = remaining_s / gap_distance;
            ratio = std::max(0.0, std::min(1.0, ratio));
            
            RefPoint interpolated;
            interpolated.x = p1.x + ratio * (p2.x - p1.x);
            interpolated.y = p1.y + ratio * (p2.y - p1.y);
            interpolated.s = s;
            interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
            interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
            interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
            interpolated.width_left = p1.width_left + ratio * (p2.width_left - p1.width_left);
            interpolated.width_right = p1.width_right + ratio * (p2.width_right - p1.width_right);
            
            return interpolated;
        }
        
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
    interpolated.width_left = p1.width_left + ratio * (p2.width_left - p1.width_left);
    interpolated.width_right = p1.width_right + ratio * (p2.width_right - p1.width_right);
    
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
    
    // Calculate available space on each side
    double space_to_left = left_boundary - current_d;
    double space_to_right = current_d - right_boundary;
    
    // Safety margins
    double safety_margin = config.safety_margin;
    double max_left = std::min(config.max_lateral_offset, left_boundary - safety_margin);
    double max_right = std::min(config.max_lateral_offset, std::abs(right_boundary) - safety_margin);
    
    // Generate offsets with adaptive sampling density
    // Rule: Dense sampling on narrow side, sparse sampling on wide side
    
    // Left side offsets (negative values)
    double left_step = config.lateral_step;
    if (space_to_left < space_to_right) {
        // Vehicle is close to left boundary -> dense left sampling
        left_step = config.lateral_step * 0.5; // Denser sampling
    } else {
        // Vehicle is close to right boundary -> sparse left sampling  
        left_step = config.lateral_step * 1.5; // Sparser sampling
    }
    
    for (double offset = -max_left; offset < current_d; offset += left_step) {
        if (offset >= right_boundary + safety_margin) {
            offsets.push_back(offset);
        }
    }
    
    // Always include current position
    offsets.push_back(current_d);
    
    // Right side offsets (positive values)
    double right_step = config.lateral_step;
    if (space_to_right < space_to_left) {
        // Vehicle is close to right boundary -> dense right sampling
        right_step = config.lateral_step * 0.5; // Denser sampling
    } else {
        // Vehicle is close to left boundary -> sparse right sampling
        right_step = config.lateral_step * 1.5; // Sparser sampling
    }
    
    for (double offset = current_d + right_step; offset <= max_right; offset += right_step) {
        if (offset <= left_boundary - safety_margin) {
            offsets.push_back(offset);
        }
    }
    
    // Sort offsets from left to right for indexing consistency
    std::sort(offsets.begin(), offsets.end());
    
    return offsets;
}

double Utils::calculate_obstacle_speed_factor(const std::vector<CartesianPoint>& path_points,
                                             const std::vector<Obstacle>& obstacles,
                                             const PlannerConfig& config) {
    if (obstacles.empty() || path_points.empty()) {
        return 1.0; // No speed reduction needed
    }
    
    // Find minimum distance from path to any obstacle
    double min_obstacle_distance = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : obstacles) {
        for (const auto& point : path_points) {
            double distance = calculate_distance(Point2D(point.x, point.y), Point2D(obstacle.x, obstacle.y));
            min_obstacle_distance = std::min(min_obstacle_distance, distance);
        }
    }
    
    // If obstacle is far away, no speed reduction
    if (min_obstacle_distance >= config.hazard_detection_distance) {
        return 1.0;
    }
    
    // Use sigmoid function for smooth speed reduction
    // Sigmoid maps: hazard_detection_distance -> 1.0 (no reduction)
    //               min_hazard_distance -> min_speed_ratio (maximum reduction)
    
    // Normalize distance to [0, 1] range
    double normalized_distance = (min_obstacle_distance - config.min_hazard_distance) / 
                                (config.hazard_detection_distance - config.min_hazard_distance);
    normalized_distance = std::max(0.0, std::min(1.0, normalized_distance));
    
    // Apply sigmoid function: sigmoid(steepness * (x - 0.5)) 
    // Shifted and scaled to map [0,1] -> [min_speed_ratio, 1.0]
    double sigmoid_input = config.speed_sigmoid_steepness * (normalized_distance - 0.5);
    double sigmoid_output = 1.0 / (1.0 + std::exp(-sigmoid_input));
    
    // Scale sigmoid output to speed range
    double speed_factor = config.min_speed_ratio + 
                         (1.0 - config.min_speed_ratio) * sigmoid_output;
    
    return speed_factor;
}

std::vector<CartesianPoint> Utils::generate_lattice_path(
    const Point2D& start_pos, double start_yaw, double lateral_offset,
    const std::vector<RefPoint>& reference_path, const PlannerConfig& config) {
    
    std::vector<CartesianPoint> path;
    
    // Use minimum planning distance to ensure adequate lookahead
    double planning_distance = std::max(config.min_planning_distance, 
                                       config.max_velocity * config.planning_horizon);
    int num_points = static_cast<int>(planning_distance / config.path_resolution);
    
    // Get initial Frenet coordinates of the vehicle
    FrenetPoint vehicle_frenet = cartesian_to_frenet(start_pos, reference_path);
    
    for (int i = 0; i < num_points; ++i) {
        double progress = i * config.path_resolution;
        
        CartesianPoint cart_point;
        
        if (i == 0) {
            // First point starts at vehicle position
            cart_point.x = start_pos.x;
            cart_point.y = start_pos.y;
            cart_point.yaw = start_yaw;
        } else {
            // Improved transition: use smoother function and longer transition distance
            double transition_distance = config.transition_smoothness; // 5 meters default
            double transition_factor;
            
            if (progress <= transition_distance) {
                // Use smooth sigmoid transition instead of sharp tanh
                double normalized_progress = progress / transition_distance;
                transition_factor = 0.5 * (1.0 + std::tanh(4.0 * (normalized_progress - 0.5)));
            } else {
                transition_factor = 1.0;
            }
            
            // Calculate target s-coordinate along reference path
            double target_s = vehicle_frenet.s + progress;
            
            // Smooth lateral transition from current offset to target offset
            double current_lateral_offset = vehicle_frenet.d;
            double target_lateral = lateral_offset;
            double smooth_lateral = current_lateral_offset + 
                                  transition_factor * (target_lateral - current_lateral_offset);
            
            // Create Frenet point and convert to Cartesian
            FrenetPoint frenet_point;
            frenet_point.s = target_s;
            frenet_point.d = smooth_lateral;
            
            CartesianPoint ref_point = frenet_to_cartesian(frenet_point, reference_path);
            cart_point = ref_point;
            
            // Smooth heading transition
            RefPoint ref_at_s = interpolate_reference_point(reference_path, target_s);
            double target_heading = ref_at_s.heading;
            cart_point.yaw = start_yaw + transition_factor * 
                            normalize_angle(target_heading - start_yaw);
        }
        
        // Get reference point for speed and curvature profile
        FrenetPoint current_frenet = cartesian_to_frenet(Point2D(cart_point.x, cart_point.y), reference_path);
        RefPoint ref_at_s = interpolate_reference_point(reference_path, current_frenet.s);
        
        // Use reference path's speed profile with gentle reduction for high curvature paths
        double curvature_penalty = std::abs(lateral_offset) * config.curvature_weight * 0.3; // Reduce penalty impact
        cart_point.velocity = ref_at_s.velocity * (1.0 - curvature_penalty);
        cart_point.velocity = std::max(cart_point.velocity, ref_at_s.velocity * 0.8); // More reasonable minimum: 80% of reference speed
        
        cart_point.curvature = ref_at_s.curvature;
        cart_point.time = i * config.dt;
        
        path.push_back(cart_point);
    }
    
    return path;
}

double Utils::calculate_path_cost(const PathCandidate& path, 
                                 const std::vector<Obstacle>& obstacles,
                                 const std::vector<RefPoint>& reference_path,
                                 const PlannerConfig& config) {
    if (path.points.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    // Algorithm: cost = lateral_offset + (1 / obstacle_distance) + obstacle_side_penalty
    double cost = 0.0;
    
    // 1. Lateral Offset Calculation:
    // For reference path: lateral offset = 0.0
    // For all other paths: lateral offset = abs(d_offset) from reference path
    double lateral_offset = std::abs(path.lateral_offset);
    cost += lateral_offset;
    
    // 2. Enhanced Obstacle Distance Calculation with Side-Awareness:
    if (!obstacles.empty()) {
        double min_obstacle_distance = std::numeric_limits<double>::max();
        double obstacle_side_penalty = 0.0;
        
        // Find closest obstacle and determine its lateral position
        Point2D closest_obstacle_pos;
        for (const auto& obstacle : obstacles) {
            double min_dist_to_path = std::numeric_limits<double>::max();
            
            for (const auto& point : path.points) {
                double distance = calculate_distance(Point2D(point.x, point.y), Point2D(obstacle.x, obstacle.y));
                min_dist_to_path = std::min(min_dist_to_path, distance);
            }
            
            if (min_dist_to_path < min_obstacle_distance) {
                min_obstacle_distance = min_dist_to_path;
                closest_obstacle_pos.x = obstacle.x;
                closest_obstacle_pos.y = obstacle.y;
            }
        }
        
        // Calculate side-aware penalty using proper reference path
        if (min_obstacle_distance < 5.0) { // Only apply side penalty for nearby obstacles
            // Convert obstacle position to Frenet coordinates to get its lateral position
            FrenetPoint obstacle_frenet = cartesian_to_frenet(closest_obstacle_pos, reference_path);
            double obstacle_lateral_offset = obstacle_frenet.d;
            double path_lateral_offset = path.lateral_offset;
            
            // Smart side-aware penalty:
            // If obstacle is on right (negative d) and path goes left (positive d) -> bonus
            // If obstacle is on left (positive d) and path goes right (negative d) -> bonus  
            // If obstacle and path are on same side -> penalty
            if ((obstacle_lateral_offset > 0 && path_lateral_offset > 0) || 
                (obstacle_lateral_offset < 0 && path_lateral_offset < 0)) {
                // Same side: penalty (dangerous - path goes toward obstacle side)
                obstacle_side_penalty = 0.8;
            } else if ((obstacle_lateral_offset > 0 && path_lateral_offset < 0) ||
                      (obstacle_lateral_offset < 0 && path_lateral_offset > 0)) {
                // Opposite side: bonus (safe - path goes away from obstacle)
                obstacle_side_penalty = -0.4;
            }
        }
        
        // Add inverse distance cost: (1 / obstacle_distance)
        if (min_obstacle_distance > 0.01) { // Avoid division by zero
            cost += (1.0 / min_obstacle_distance);
        } else {
            cost += 1000.0; // Very high cost for extremely close obstacles
        }
        
        // Add side-aware penalty
        cost += obstacle_side_penalty;
    }
    
    return cost;
}

crazy_planner_msgs::msg::WaypointArray Utils::convert_to_waypoint_array(const PathCandidate& path, const std::vector<RefPoint>& reference_path) {
    crazy_planner_msgs::msg::WaypointArray waypoint_array;
    
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& point = path.points[i];
        
        crazy_planner_msgs::msg::Waypoint waypoint;
        waypoint.id = static_cast<int32_t>(i);
        
        // Set map coordinates
        waypoint.x_m = point.x;
        waypoint.y_m = point.y;
        waypoint.psi_rad = point.yaw;
        waypoint.vx_mps = point.velocity;
        waypoint.kappa_radpm = point.curvature;
        
        // Calculate acceleration (simple finite difference)
        if (i > 0) {
            double dt = 0.1; // Approximate time step
            double dv = point.velocity - path.points[i-1].velocity;
            waypoint.ax_mps2 = dv / dt;
        } else {
            waypoint.ax_mps2 = 0.0;
        }
        
        // Convert to Frenet coordinates using actual path progression
        // Calculate cumulative arc length
        if (i == 0) {
            waypoint.s_m = 0.0;
        } else {
            double dx = point.x - path.points[i-1].x;
            double dy = point.y - path.points[i-1].y;
            double ds = std::sqrt(dx*dx + dy*dy);
            waypoint.s_m = waypoint_array.waypoints[i-1].s_m + ds;
        }
        
        // Lateral offset from path generation (more accurate than fixed value)
        waypoint.d_m = path.lateral_offset;
        
        // Get track bounds from reference path if available
        if (!reference_path.empty()) {
            // Find closest reference point and interpolate track boundaries
            FrenetPoint frenet_point = cartesian_to_frenet(Point2D(point.x, point.y), reference_path);
            RefPoint ref_point = interpolate_reference_point(reference_path, frenet_point.s);
            waypoint.d_right = ref_point.width_right;
            waypoint.d_left = ref_point.width_left;
            
        } else {
            // Fallback to varying default values
            waypoint.d_right = 1.8 + 0.2 * std::sin(waypoint.s_m * 0.1);
            waypoint.d_left = 1.8 + 0.2 * std::cos(waypoint.s_m * 0.1);
            
        }
        
        waypoint_array.waypoints.push_back(waypoint);
    }
    
    return waypoint_array;
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

} // namespace lattice_planner_pkg