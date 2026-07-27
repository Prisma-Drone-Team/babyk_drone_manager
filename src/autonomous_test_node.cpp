#include "babyk_drone_manager/autonomous_test_node.h"

using namespace std::chrono_literals;

AutonomousTestNode::AutonomousTestNode() 
    : Node("autonomous_test_node"),
      current_status_("UNKNOWN"),
      last_command_was_land_(false),
      system_initialized_(false),
      consecutive_failures_(0),
      last_command_sent_(""),
      has_odometry_(false),
      exploration_goal_counter_(0),
      gen_(rd_()),
      command_dist_(0, 99)
{
    // Declare and get parameters
    this->declare_parameter("command_interval_min", 120);  // 2 minutes
    this->declare_parameter("command_interval_max", 180);  // 3 minutes
    this->declare_parameter("max_wait_time", 60);          // 1 minute max wait
    this->declare_parameter("land_probability", 0.20);     // 20% chance of landing
    this->declare_parameter("max_consecutive_failures", 3); // Max failures before emergency land
    this->declare_parameter("max_goal_distance", 5.0);      // Max forward distance for exploration
    this->declare_parameter("goal_distance_ratio", 0.5);    // Ratio of max distance to spawn goal
    this->declare_parameter("min_forward_space", 1.5);      // Minimum forward space required to avoid landing
    
    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_ = this->get_parameter("max_wait_time").as_int();
    land_probability_ = this->get_parameter("land_probability").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    max_goal_distance_ = this->get_parameter("max_goal_distance").as_double();
    goal_distance_ratio_ = this->get_parameter("goal_distance_ratio").as_double();
    min_forward_space_ = this->get_parameter("min_forward_space").as_double();
    
    // Available goals (goal1 to goal7)
    available_goals_ = {"goal1", "goal2", "goal3", "goal4", "goal5", "goal6", "goal7"};

    // Create publisher and subscriber
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/seed_pdt_drone/command", 10);
    
    status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/trajectory_interpolator/status", 10,
        std::bind(&AutonomousTestNode::status_callback, this, std::placeholders::_1));
        
    // Set up SensorData QoS for odometry and octomap
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();
    
    octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", sensor_qos,
        std::bind(&AutonomousTestNode::octomap_callback, this, std::placeholders::_1));
        
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", sensor_qos,
        std::bind(&AutonomousTestNode::odometry_callback, this, std::placeholders::_1));
        
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create timer for sending commands
    command_timer_ = this->create_wall_timer(
        5s, std::bind(&AutonomousTestNode::command_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Autonomous Test Node initialized");
    RCLCPP_INFO(this->get_logger(), "Available goals: goal1-goal7");
    RCLCPP_INFO(this->get_logger(), "Command interval: %d-%d seconds", 
                command_interval_min_, command_interval_max_);
    RCLCPP_INFO(this->get_logger(), "Land probability: %.1f%%", land_probability_ * 100);
    RCLCPP_INFO(this->get_logger(), "Max consecutive failures before emergency land: %d", max_consecutive_failures_);
    RCLCPP_INFO(this->get_logger(), "Max goal distance: %.1fm", max_goal_distance_);
}

void AutonomousTestNode::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string previous_status = current_status_;
    current_status_ = msg->data;
    
    // Track failures: if status changed and we detect a failure condition
    if (previous_status != current_status_) {
        // Check if this represents a failure (you can customize these conditions)
        if (current_status_ == "IDLE" && !last_command_sent_.empty()) {
            // Command completed successfully - reset failure counter
            if (consecutive_failures_ > 0) {
                RCLCPP_INFO(this->get_logger(), "Command '%s' completed successfully. Resetting failure counter.", 
                           last_command_sent_.c_str());
                consecutive_failures_ = 0;
            }
        }
        else if (current_status_.find("ERROR") != std::string::npos || 
                 current_status_.find("FAILED") != std::string::npos) {
            // Command failed - increment failure counter
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), "Command '%s' failed with status '%s'. Consecutive failures: %d/%d", 
                       last_command_sent_.c_str(), current_status_.c_str(), 
                       consecutive_failures_, max_consecutive_failures_);
        }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Status: %s", current_status_.c_str());
}

void AutonomousTestNode::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
    if (tree) {
        octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }
    
    if (octree_) {
        static bool first_time = true;
        if (first_time || octomap_frame_id_ != msg->header.frame_id) {
            octomap_frame_id_ = msg->header.frame_id;
            RCLCPP_WARN(this->get_logger(), "🔥 OCTOMAP FRAME ID IS: '%s' 🔥", octomap_frame_id_.c_str());
            first_time = false;
        }
    }
}

void AutonomousTestNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pos_(0) = msg->pose.pose.position.x;
    current_pos_(1) = msg->pose.pose.position.y;
    current_pos_(2) = msg->pose.pose.position.z;
    has_odometry_ = true;
}

std::optional<Eigen::Vector3d> AutonomousTestNode::find_frontier_goal(bool& should_return)
{
    should_return = false;
    if (!octree_) {
        RCLCPP_WARN(this->get_logger(), "Octree is null!");
        return std::nullopt;
    }
    if (octree_->size() == 0) {
        RCLCPP_WARN(this->get_logger(), "Octree is empty (size 0)!");
        return std::nullopt;
    }
    if (!has_odometry_) {
        RCLCPP_WARN(this->get_logger(), "No odometry received yet!");
        return std::nullopt;
    }

    // 1. Get true physical drone pose in 'map' frame
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get map->base_link for frontier search: %s", ex.what());
        return std::nullopt;
    }
    
    // 2. Transform the drone pose to the octomap frame
    std::string target_frame = octomap_frame_id_.empty() ? "map" : octomap_frame_id_;
    
    geometry_msgs::msg::PoseStamped base_in_map;
    base_in_map.header.frame_id = "map";
    base_in_map.header.stamp = rclcpp::Time(0);
    base_in_map.pose.position.x = map_to_base.transform.translation.x;
    base_in_map.pose.position.y = map_to_base.transform.translation.y;
    base_in_map.pose.position.z = map_to_base.transform.translation.z;
    base_in_map.pose.orientation = map_to_base.transform.rotation;
    
    geometry_msgs::msg::PoseStamped base_in_octomap;
    try {
        base_in_octomap = tf_buffer_->transform(base_in_map, target_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF error map->%s: %s", target_frame.c_str(), ex.what());
        return std::nullopt;
    }

    // 3. Extract coordinates in octomap frame
    double drone_x = base_in_octomap.pose.position.x;
    double drone_y = base_in_octomap.pose.position.y;

    // Calculate drone's forward direction in octomap frame using quaternion yaw
    double qx = base_in_octomap.pose.orientation.x;
    double qy = base_in_octomap.pose.orientation.y;
    double qz = base_in_octomap.pose.orientation.z;
    double qw = base_in_octomap.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    double forward_x = std::cos(yaw);
    double forward_y = std::sin(yaw);

    int total_nodes = 0;
    int free_nodes = 0;
    for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
        total_nodes++;
        if (!octree_->isNodeOccupied(*it)) {
            free_nodes++;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Octomap stats: %d total nodes, %d FREE nodes.", total_nodes, free_nodes);

    double max_dist_forward = -1.0;
    bool found = false;
    double tunnel_width = 5.0; 
    
    // Pass 1: Find the absolute furthest reach (max_dist_forward) along the drone's heading
    for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            double x = it.getX();
            double y = it.getY();
            
            double dx = x - drone_x;
            double dy = y - drone_y;
            
            // Project the distance along the forward vector (dot product)
            double forward_dist = dx * forward_x + dy * forward_y;
            
            // Calculate lateral distance to keep it inside the tunnel
            double lateral_dist = std::abs(dx * (-forward_y) + dy * forward_x);
            
            // Only look forward (forward_dist > 0) and within lateral tunnel
            if (forward_dist > 0 && forward_dist <= max_goal_distance_ && lateral_dist <= tunnel_width) {
                if (!found || forward_dist > max_dist_forward) {
                    max_dist_forward = forward_dist;
                    found = true;
                }
            }
        }
    }

    if (found && max_dist_forward >= min_forward_space_) { // Require at least min_forward_space_ of forward free space
        double min_lateral = 10000.0;
        double max_lateral = -10000.0;
        double sum_z = 0.0;
        int count = 0;
        
        // Pass 2: Bounding box midpoint ONLY of the nodes at the very edge of the frontier
        // This keeps the drone perfectly centered even if the visual map is asymmetric!
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();
                
                double dx = x - drone_x;
                double dy = y - drone_y;
                double forward_dist = dx * forward_x + dy * forward_y;
                // Use signed lateral distance to find left/right bounds
                double lateral_pos = dx * (-forward_y) + dy * forward_x;
                
                // Only consider nodes in the front "slice" of the frontier
                if (forward_dist >= max_dist_forward - 3.0 && forward_dist <= max_dist_forward && std::abs(lateral_pos) <= tunnel_width) {
                    if (lateral_pos < min_lateral) min_lateral = lateral_pos;
                    if (lateral_pos > max_lateral) max_lateral = lateral_pos;
                    sum_z += z;
                    count++;
                }
            }
        }
            
        if (count > 0) {
            double avg_lateral = (min_lateral + max_lateral) / 2.0;
            
            // Lessen the damping to allow more lateral exploration
            avg_lateral *= 0.8;
            
            // Add some randomness to make it less straight (between -1.0 and 1.0 meters)
            std::uniform_real_distribution<double> lat_dist(-1.0, 1.0);
            avg_lateral += lat_dist(gen_);
            
            double avg_z = sum_z / count;
            
            // Reconstruct X and Y from forward and lateral components
            // We want the goal to be based on the parameterizable ratio
            double target_forward = max_dist_forward * goal_distance_ratio_;
            
            double goal_x = drone_x + target_forward * forward_x + avg_lateral * (-forward_y);
            double goal_y = drone_y + target_forward * forward_y + avg_lateral * forward_x;
            
            // Constrain Z to reasonable flight altitudes
            if (avg_z < 0.5) avg_z = 0.5;
            if (avg_z > 2.0) avg_z = 2.0;
            
            // Transform the goal back from octomap frame to 'map'
            geometry_msgs::msg::PoseStamped goal_in_octomap;
            goal_in_octomap.header.frame_id = target_frame;
            goal_in_octomap.header.stamp = rclcpp::Time(0);
            goal_in_octomap.pose.position.x = goal_x;
            goal_in_octomap.pose.position.y = goal_y;
            goal_in_octomap.pose.position.z = avg_z;
            goal_in_octomap.pose.orientation.w = 1.0;
            
            geometry_msgs::msg::PoseStamped goal_in_map;
            try {
                goal_in_map = tf_buffer_->transform(goal_in_octomap, "map", tf2::durationFromSec(0.1));
                
                RCLCPP_INFO(this->get_logger(), "🎯 FOUND FRONTIER! Octomap coords: [%.2f, %.2f, %.2f] -> Map coords: [%.2f, %.2f, %.2f]",
                            goal_x, goal_y, avg_z,
                            goal_in_map.pose.position.x, goal_in_map.pose.position.y, goal_in_map.pose.position.z);
                            
                return Eigen::Vector3d(goal_in_map.pose.position.x, goal_in_map.pose.position.y, goal_in_map.pose.position.z);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF error %s->map: %s", target_frame.c_str(), ex.what());
                return std::nullopt;
            }
        }
    } else {
        if (!found) {
            RCLCPP_WARN(this->get_logger(), "Octomap has nodes, but NO free space was found in the forward direction (yaw: %.2f)!", yaw);
            // If no free space is found at all, we might be blocked completely
            should_return = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Octomap found free space, but max distance is %.2fm (must be > %.2fm)!", max_dist_forward, min_forward_space_);
            if (max_dist_forward < min_forward_space_) {
                RCLCPP_WARN(this->get_logger(), "Forward space is too small! Triggering return sequence.");
                should_return = true;
            }
        }
    }

    return std::nullopt;
}

void AutonomousTestNode::send_explore_flyto()
{
    bool should_return = false;
    auto goal_opt = find_frontier_goal(should_return);
    
    if (should_return) {
        RCLCPP_WARN(this->get_logger(), "Exploration logic determined space is insufficient. Returning to initial point (goal1).");
        send_command("flyto(goal1)");
        return;
    }
    
    if (!goal_opt) {
        RCLCPP_WARN(this->get_logger(), "No frontier found in Octomap. Waiting for free space to be detected...");
        return;
    }
    
    Eigen::Vector3d goal = *goal_opt;
    exploration_goal_counter_++;
    std::string frame_name = "exploration_goal_" + std::to_string(exploration_goal_counter_);
    
    // The goal returned by find_frontier_goal() is already in 'map' frame!
    
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get map->base_link: %s", ex.what());
        return;
    }
    
    double yaw_map = std::atan2(goal(1) - map_to_base.transform.translation.y, goal(0) - map_to_base.transform.translation.x);
    
    geometry_msgs::msg::PoseStamped goal_map_pose;
    goal_map_pose.header.frame_id = "map";
    goal_map_pose.header.stamp = rclcpp::Time(0);
    goal_map_pose.pose.position.x = goal(0);
    goal_map_pose.pose.position.y = goal(1);
    goal_map_pose.pose.position.z = goal(2);
    goal_map_pose.pose.orientation.z = std::sin(yaw_map / 2.0);
    goal_map_pose.pose.orientation.w = std::cos(yaw_map / 2.0);
    
    // Publish a dynamic TF frame for the goal
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map"; 
    t.child_frame_id = frame_name;
    
    t.transform.translation.x = goal_map_pose.pose.position.x;
    t.transform.translation.y = goal_map_pose.pose.position.y;
    t.transform.translation.z = goal_map_pose.pose.position.z;
    t.transform.rotation = goal_map_pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(t);
    
    // Send the flyto command
    std::string command = "flyto(" + frame_name + ")";
    send_command(command);
    
    RCLCPP_INFO(this->get_logger(), "🚀 EXPLORING: Published frontier TF %s at [%.2f, %.2f, %.2f]", 
                frame_name.c_str(), goal(0), goal(1), goal(2));
}

void AutonomousTestNode::command_timer_callback()
{
    static auto last_command_time = this->now();
    static auto last_status_change = this->now();
    
    auto current_time = this->now();
    
    // Initial takeoff after system starts
    if (!system_initialized_) {
        if (is_system_idle()) {
            RCLCPP_INFO(this->get_logger(), "System ready, sending initial takeoff");
            send_takeoff();
            system_initialized_ = true;
            last_command_time = current_time;
            last_status_change = current_time;
            return;
        }
        return;
    }
    
    // Update last status change time if status changed
    static std::string previous_status = current_status_;
    if (current_status_ != previous_status) {
        last_status_change = current_time;
        previous_status = current_status_;
    }
    
    // Calculate time intervals
    auto time_since_last_command = (current_time - last_command_time).seconds();
    auto time_since_status_change = (current_time - last_status_change).seconds();
    
    // Generate random interval for next command
    std::uniform_int_distribution<int> interval_dist(command_interval_min_, command_interval_max_);
    int target_interval = interval_dist(gen_);
    
    // Check if we should send a new command
    bool should_send_command = false;
    
    if (is_system_idle()) {
        // If system just became idle, wait 2 seconds to ensure it's not a transient state (like STOP before TELEOP)
        // Otherwise wait for the interval
        if (time_since_last_command >= 10.0 && time_since_status_change >= 2.0) {  // Minimum 10 seconds between commands, and wait 2s
            should_send_command = true;
        }
    }
    else if (time_since_status_change >= max_wait_time_ && time_since_last_command >= target_interval) {
        // System is stuck, force a new command
        should_send_command = true;
        RCLCPP_WARN(this->get_logger(), "Forcing new command: system stuck in '%s' for %.1f seconds", 
                    current_status_.c_str(), time_since_status_change);
    }
    
    if (should_send_command) {
        // EMERGENCY LAND: Too many consecutive failures
        if (consecutive_failures_ >= max_consecutive_failures_ && !last_command_was_land_ && last_command_sent_ != "land") {
            RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY LAND: %d consecutive failures detected! Forcing emergency land command.", 
                         consecutive_failures_);
            send_land();
            last_command_was_land_ = true;
            consecutive_failures_ = 0; // Reset counter after emergency land
            RCLCPP_ERROR(this->get_logger(), "🚨 Emergency land sent - next command will MANDATORY be takeoff");
        }
        // If last command was land, next MUST be takeoff (CRITICAL RULE)
        else if (last_command_was_land_ || last_command_sent_ == "land") {
            send_takeoff();
            last_command_was_land_ = false;
            RCLCPP_INFO(this->get_logger(), "MANDATORY takeoff after land command");
        }
        else {
            // Random choice between explore and land
            double rand_val = static_cast<double>(command_dist_(gen_)) / 100.0;
            
            if (rand_val < land_probability_) {
                // Send land command
                send_land();
                last_command_was_land_ = true;
                RCLCPP_INFO(this->get_logger(), "Land command sent - next command will be takeoff");
            } else {
                // Use the new exploration function
                send_explore_flyto();
                last_command_was_land_ = false;
            }
        }
        
        last_command_time = current_time;
        last_status_change = current_time;
    }
}

void AutonomousTestNode::send_command(const std::string& command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    command_publisher_->publish(msg);
    
    // Smart logging with context-aware repetition detection
    if (!last_command_sent_.empty() && command != last_command_sent_) {
        RCLCPP_INFO(this->get_logger(), "✅ Sent command: '%s' (changed from '%s')", 
                    command.c_str(), last_command_sent_.c_str());
    } 
    else if (!last_command_sent_.empty() && command == last_command_sent_) {
        // Check if this is a legitimate repetition
        bool is_legitimate_repeat = false;
        std::string reason = "";
        
        if (command == "takeoff") {
            is_legitimate_repeat = true;
            reason = "retry after failure or after land";
        }
        else if (command == "land") {
            is_legitimate_repeat = false; // Land should never be repeated
            reason = "INVALID - land should not repeat";
        }
        else if (command.substr(0, 5) == "flyto") {
            is_legitimate_repeat = true; // Could be retry after failure
            reason = "retry after failure";
        }
        
        if (is_legitimate_repeat) {
            RCLCPP_INFO(this->get_logger(), "🔄 Sent command: '%s' (repeated - %s)", 
                        command.c_str(), reason.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ Sent command: '%s' (INVALID REPETITION - %s)", 
                        command.c_str(), reason.c_str());
        }
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "🚀 Sent command: '%s' (first command)", command.c_str());
    }
    
    // Update the last command tracking
    last_command_sent_ = command;
}

void AutonomousTestNode::send_takeoff()
{
    send_command("takeoff");
}

void AutonomousTestNode::send_land()
{
    send_command("land");
}

void AutonomousTestNode::send_random_flyto()
{
    std::string new_command;
    int max_attempts = 20; // Safety limit to avoid infinite loops
    int attempts = 0;

    do {
        // Pick a random goal from goal1 to goal7
        std::uniform_int_distribution<int> goal_dist(0, available_goals_.size() - 1);
        int goal_index = goal_dist(gen_);

        // Probabilità 10% per cover/circle, 90% flyto normale
        double special_prob = static_cast<double>(command_dist_(gen_)) / 100.0;
        if (special_prob < 0.05) {
            // new_command = "flyto(cover(" + available_goals_[goal_index] + ",2.0,5.0))";
            new_command = "cover((1,1),(1,4),(4,4),(4,1))"; // new standard for area covering
        } else if (special_prob < 0.10) {
            new_command = "flyto(circle(" + available_goals_[goal_index] + "))";
        } else {
            new_command = "flyto(" + available_goals_[goal_index] + ")";
        }

        attempts++;

        if (attempts >= max_attempts) {
            RCLCPP_WARN(this->get_logger(), "Could not find different goal after %d attempts, allowing repetition", max_attempts);
            break;
        }

    } while (new_command == last_command_sent_ && available_goals_.size() > 1);

    send_command(new_command);

    if (attempts > 1) {
        RCLCPP_INFO(this->get_logger(), "Avoided repeating command, selected different goal after %d attempts", attempts);
    }
}

bool AutonomousTestNode::is_system_idle()
{
    return current_status_ == "IDLE" || 
           current_status_ == "STOPPED" || 
           current_status_ == "MISSION_COMPLETED";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AutonomousTestNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting Autonomous Test Node");
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}