#include "babyk_drone_manager/move_manager_node.h"
#include <iostream>
#include <thread>
#include <chrono>

MoveManagerNode::MoveManagerNode() 
    : Node("move_manager_node"),
      odometry_received_(false),
      running_(true),
      path_planner_status_("IDLE"),
      traj_interp_status_("IDLE"),
      overall_status_("IDLE") {
    
    // Declare parameters
    declare_parameters();

    // Get parameters
    command_topic_ = this->get_parameter("command_topic").as_string();
    status_topic_ = this->get_parameter("status_topic").as_string();
    path_planner_goal_topic_ = this->get_parameter("path_planner_goal_topic").as_string();
    path_planner_status_topic_ = this->get_parameter("path_planner_status_topic").as_string();
    traj_interp_path_topic_ = this->get_parameter("traj_interp_path_topic").as_string();
    traj_interp_status_topic_ = this->get_parameter("traj_interp_status_topic").as_string();
    planned_path_topic_ = this->get_parameter("planned_path_topic").as_string();
    odometry_topic_ = this->get_parameter("odometry_topic").as_string();
    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    simulation_mode_ = this->get_parameter("simulation").as_bool();

    RCLCPP_INFO(get_logger(), "Move Manager Node initialized");
    RCLCPP_INFO(get_logger(), "  Command topic: %s", command_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Takeoff altitude: %.2f m", takeoff_altitude_);
    RCLCPP_INFO(get_logger(), "  Simulation mode: %s", simulation_mode_ ? "enabled" : "disabled");

    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Initialize TF2 for simulation if needed
    if (simulation_mode_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // Timer for TF publishing at 100Hz
        tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), 
            std::bind(&MoveManagerNode::timer_tf_callback, this));
        
        RCLCPP_INFO(get_logger(), "Simulation TF publishing enabled at 100Hz");
    }

    // Initialize publishers
    path_planner_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        path_planner_goal_topic_, 10);
    
    traj_interp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        traj_interp_path_topic_, 10);
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, 10);

    // Initialize subscribers
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        command_topic_, 10, std::bind(&MoveManagerNode::command_callback, this, std::placeholders::_1));

    path_planner_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        path_planner_status_topic_, 10, std::bind(&MoveManagerNode::path_planner_status_callback, this, std::placeholders::_1));

    traj_interp_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        traj_interp_status_topic_, 10, std::bind(&MoveManagerNode::traj_interp_status_callback, this, std::placeholders::_1));

    planned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        planned_path_topic_, 10, std::bind(&MoveManagerNode::planned_path_callback, this, std::placeholders::_1));

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, qos, std::bind(&MoveManagerNode::odometry_callback, this, std::placeholders::_1));

    // Start command processor thread
    command_processor_thread_ = std::thread(&MoveManagerNode::command_processor, this);

    RCLCPP_INFO(get_logger(), "Move Manager Node ready to accept commands");
}

MoveManagerNode::~MoveManagerNode() {
    running_ = false;
    if (command_processor_thread_.joinable()) {
        command_processor_thread_.join();
    }
}

void MoveManagerNode::declare_parameters() {
    // Topic parameters
    this->declare_parameter("command_topic", "/move_manager/command");
    this->declare_parameter("status_topic", "/move_manager/status");
    this->declare_parameter("path_planner_goal_topic", "/move_base_simple/goal");
    this->declare_parameter("path_planner_status_topic", "/path_planner/status");
    this->declare_parameter("traj_interp_path_topic", "/trajectory_path");
    this->declare_parameter("traj_interp_status_topic", "/trajectory_interpolator/status");
    this->declare_parameter("planned_path_topic", "/trajectory_path");
    this->declare_parameter("odometry_topic", "/px4/odometry/out");

    // Flight parameters
    this->declare_parameter("takeoff_altitude", 1.5);
    
    // Simulation parameter
    this->declare_parameter("simulation", false);
}

void MoveManagerNode::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    received_command_ = msg->data;
    RCLCPP_INFO(get_logger(), "Received command: %s", received_command_.c_str());
}

void MoveManagerNode::path_planner_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    path_planner_status_ = msg->data;
    update_overall_status();
}

void MoveManagerNode::traj_interp_status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    traj_interp_status_ = msg->data;
    update_overall_status();
}

void MoveManagerNode::planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    // Solo inoltrare il percorso se stiamo aspettando un percorso pianificato
    // (evita loop infiniti di re-publishing)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (overall_status_ == "PLANNING_PATH") {
            RCLCPP_INFO(get_logger(), "Received planned path with %zu waypoints, forwarding to traj_interp", 
                        msg->poses.size());
            
            // Forward the planned path directly to traj_interp
            traj_interp_path_pub_->publish(*msg);
            
            // Aggiorna lo stato per indicare che il percorso è stato inoltrato
            overall_status_ = "PATH_FORWARDED";
        } else {
            RCLCPP_DEBUG(get_logger(), "Ignoring planned path (status: %s) - not expecting new path", 
                        overall_status_.c_str());
        }
    }
}

void MoveManagerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_pose_ = msg->pose.pose;
    
    // Publish TF in simulation mode
    if (simulation_mode_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        
        // Set the header
        transform_stamped.header.stamp = msg->header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";

        // Set translation (position)
        transform_stamped.transform.translation.x = msg->pose.pose.position.x;
        transform_stamped.transform.translation.y = msg->pose.pose.position.y;
        transform_stamped.transform.translation.z = msg->pose.pose.position.z;

        transform_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
        transform_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
        transform_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
        transform_stamped.transform.rotation.w = msg->pose.pose.orientation.w;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    
    if (!odometry_received_) {
        odometry_received_ = true;
        RCLCPP_INFO(get_logger(), "First odometry received: [%.3f, %.3f, %.3f]", 
                    current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
    }
}

void MoveManagerNode::command_processor() {
    while (running_ && rclcpp::ok()) {
        std::string command_to_process;
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (current_command_ != received_command_ && !received_command_.empty()) {
                command_to_process = received_command_;
            }
        }
        
        if (!command_to_process.empty()) {
            std::vector<std::string> parts = parse_command(command_to_process);
            
            if (!parts.empty()) {
                // Stop any ongoing operation if new command received
                if (current_command_ != command_to_process) {
                    RCLCPP_INFO(get_logger(), "Processing new command: %s", command_to_process.c_str());
                    
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        current_command_ = command_to_process;
                    }
                    
                    process_command(parts);
                }
            }
        }
        
        // Publish status
        std_msgs::msg::String status_msg;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            status_msg.data = overall_status_;
        }
        status_pub_->publish(status_msg);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void MoveManagerNode::process_command(const std::vector<std::string>& command_parts) {
    const std::string& command = command_parts[0];
    
    if (command == "flyto") {
        handle_flyto_command(command_parts);
    } else if (command == "go") {
        handle_go_command(command_parts);
    } else if (command == "takeoff") {
        handle_takeoff_command(command_parts);
    } else if (command == "land") {
        handle_land_command(command_parts);
    } else if (command == "stop") {
        handle_stop_command();
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown command: %s", command.c_str());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_UNKNOWN_COMMAND";
    }
}

void MoveManagerNode::handle_flyto_command(const std::vector<std::string>& parts) {
    if (parts.size() < 2) {
        RCLCPP_ERROR(get_logger(), "flyto command requires a frame name: flyto(frame_name)");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_FLYTO";
        return;
    }
    
    geometry_msgs::msg::Pose target_pose;
    if (lookup_transform(parts[1], target_pose)) {
        // Send goal to path planner
        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.stamp = this->get_clock()->now();
        goal_msg.header.frame_id = "map";
        goal_msg.pose = target_pose;
        
        path_planner_goal_pub_->publish(goal_msg);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            overall_status_ = "PLANNING_PATH";
        }
        
        RCLCPP_INFO(get_logger(), "Sent flyto goal to path planner: [%.3f, %.3f, %.3f]", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
    } else {
        RCLCPP_ERROR(get_logger(), "Could not find transform for frame: %s", parts[1].c_str());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_FRAME_NOT_FOUND";
    }
}

void MoveManagerNode::handle_go_command(const std::vector<std::string>& parts) {
    if (parts.size() < 4) {
        RCLCPP_ERROR(get_logger(), "go command requires 3 coordinates: go(x,y,z)");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_GO";
        return;
    }
    
    try {
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = std::stod(parts[1]);
        target_pose.position.y = std::stod(parts[2]);
        target_pose.position.z = std::stod(parts[3]);
        target_pose.orientation.w = 1.0; // Default orientation
        
        // Create direct path and send to traj_interp
        nav_msgs::msg::Path direct_path = create_direct_path(target_pose);
        traj_interp_path_pub_->publish(direct_path);
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            overall_status_ = "EXECUTING_DIRECT_PATH";
        }
        
        RCLCPP_INFO(get_logger(), "Sent direct go command to traj_interp: [%.3f, %.3f, %.3f]", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Invalid coordinates in go command: %s", e.what());
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_INVALID_COORDINATES";
    }
}

void MoveManagerNode::handle_takeoff_command(const std::vector<std::string>& parts) {
    if (!odometry_received_) {
        RCLCPP_ERROR(get_logger(), "Cannot takeoff: no odometry received");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_ODOMETRY";
        return;
    }

    geometry_msgs::msg::Pose takeoff_pose;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        takeoff_pose = current_pose_;  // Keep current orientation
    }
    takeoff_pose.position.z = takeoff_altitude_;  // Only change altitude

    // Create direct path for takeoff and send to traj_interp
    nav_msgs::msg::Path takeoff_path = create_direct_path(takeoff_pose);
    traj_interp_path_pub_->publish(takeoff_path);

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "TAKING_OFF";
    }

    RCLCPP_INFO(get_logger(), "Sent takeoff command to traj_interp: altitude %.3f", takeoff_altitude_);
}

void MoveManagerNode::handle_land_command(const std::vector<std::string>& parts) {
    if (!odometry_received_) {
        RCLCPP_ERROR(get_logger(), "Cannot land: no odometry received");
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "ERROR_NO_ODOMETRY";
        return;
    }
    
    geometry_msgs::msg::Pose land_pose;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        land_pose = current_pose_;  // Keep current orientation
    }
    land_pose.position.z = 0.0; // Only change altitude to land at ground level

    // Create direct path for landing and send to traj_interp
    nav_msgs::msg::Path land_path = create_direct_path(land_pose);
    traj_interp_path_pub_->publish(land_path);
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "LANDING";
    }
    
    RCLCPP_INFO(get_logger(), "Sent land command to traj_interp");
}

void MoveManagerNode::handle_stop_command() {
    // Create empty path to stop trajectory execution
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = this->get_clock()->now();
    empty_path.header.frame_id = "map";
    
    traj_interp_path_pub_->publish(empty_path);
    
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        overall_status_ = "STOPPED";
    }
    
    RCLCPP_INFO(get_logger(), "Sent stop command");
}

std::vector<std::string> MoveManagerNode::parse_command(const std::string& command) {
    // Parse command format like "flyto(frame_name)" or "go(x,y,z)"
    std::vector<std::string> result;
    
    size_t paren_pos = command.find('(');
    if (paren_pos != std::string::npos) {
        // Extract command name
        result.push_back(command.substr(0, paren_pos));
        
        // Extract arguments
        size_t end_paren = command.find(')', paren_pos);
        if (end_paren != std::string::npos) {
            std::string args = command.substr(paren_pos + 1, end_paren - paren_pos - 1);
            std::stringstream ss(args);
            std::string item;
            
            while (std::getline(ss, item, ',')) {
                // Remove whitespace
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);
                result.push_back(item);
            }
        }
    } else {
        // Simple command without arguments
        result.push_back(command);
    }
    
    return result;
}

bool MoveManagerNode::lookup_transform(const std::string& frame_name, geometry_msgs::msg::Pose& pose) {
    try {
        geometry_msgs::msg::TransformStamped tf_stamped = 
            tf_buffer_->lookupTransform("map", frame_name, tf2::TimePointZero);
        
        pose.position.x = tf_stamped.transform.translation.x;
        pose.position.y = tf_stamped.transform.translation.y;
        pose.position.z = tf_stamped.transform.translation.z;
        pose.orientation = tf_stamped.transform.rotation;
        
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "Could not find transform from map to %s: %s", 
                    frame_name.c_str(), ex.what());
        return false;
    }
}

nav_msgs::msg::Path MoveManagerNode::create_direct_path(const geometry_msgs::msg::Pose& target_pose) {
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";
    
    // Add current position as start
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.header = path.header;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        start_pose.pose = current_pose_;
    }
    path.poses.push_back(start_pose);
    
    // Add target position as end
    geometry_msgs::msg::PoseStamped end_pose;
    end_pose.header = path.header;
    end_pose.pose = target_pose;
    path.poses.push_back(end_pose);
    
    return path;
}

void MoveManagerNode::update_overall_status() {
    // Logic to combine statuses from path planner and traj interp
    if (path_planner_status_ == "PLANNING") {
        overall_status_ = "PLANNING_PATH";
    } else if (path_planner_status_.find("FAILED") != std::string::npos) {
        overall_status_ = "PLANNING_FAILED";
    } else if (traj_interp_status_ == "FOLLOWING_TRAJECTORY") {
        overall_status_ = "EXECUTING_TRAJECTORY";
    } else if (traj_interp_status_ == "IDLE" && path_planner_status_ == "PATH_PUBLISHED" && overall_status_ != "PATH_FORWARDED") {
        overall_status_ = "TRAJECTORY_COMPLETED";
    }
    // Manteniamo lo stato PATH_FORWARDED finché traj_interp non inizia a seguire la traiettoria
    // Keep other statuses as they are
}

// Timer callback for TF publishing in simulation mode
void MoveManagerNode::timer_tf_callback() {
    if (simulation_mode_) {
        static_tf_pub();
    }
}

// Function for static transforms (from original move_manager)
void MoveManagerNode::static_tf_pub() {
    geometry_msgs::msg::TransformStamped t;
    
    // Transform: base_link -> baby_k_0/OakD-Lite/base_link/IMX214
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "baby_k_0/OakD-Lite/base_link/IMX214";

    t.transform.translation.x = 0.15;
    t.transform.translation.y = 0.03;
    t.transform.translation.z = 0.202;

    tf2::Quaternion q;
    q.setRPY(-1.5707, 0, -1.5707);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
    static_tf_broadcaster_->sendTransform(t);

    // Transform: base_link -> base_link_FRD  
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "base_link_FRD";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = -1.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 0.0;

    static_tf_broadcaster_->sendTransform(t);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
