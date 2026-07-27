#include "babyk_drone_manager/sewer_autonomous_test_node.h"

using namespace std::chrono_literals;

SewerAutonomousTestNode::SewerAutonomousTestNode() 
    : Node("sewer_autonomous_test_node"),
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
    this->declare_parameter("command_interval_min", 30);   
    this->declare_parameter("command_interval_max", 45);   
    this->declare_parameter("max_wait_time", 60);          
    this->declare_parameter("land_probability", 0.0);      // No random landing in sewer
    this->declare_parameter("max_consecutive_failures", 3); 
    this->declare_parameter("max_descent_distance", 3.0);   
    this->declare_parameter("goal_distance_ratio", 0.6);    
    this->declare_parameter("min_descent_space", 0.5);      
    this->declare_parameter("tunnel_radius", 2.0);          // Max lateral distance to consider
    
    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_ = this->get_parameter("max_wait_time").as_int();
    land_probability_ = this->get_parameter("land_probability").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    max_descent_distance_ = this->get_parameter("max_descent_distance").as_double();
    goal_distance_ratio_ = this->get_parameter("goal_distance_ratio").as_double();
    min_descent_space_ = this->get_parameter("min_descent_space").as_double();
    tunnel_radius_ = this->get_parameter("tunnel_radius").as_double();

    // Create publisher and subscriber
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/seed_pdt_drone/command", 10);
    
    status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/trajectory_interpolator/status", 10,
        std::bind(&SewerAutonomousTestNode::status_callback, this, std::placeholders::_1));
        
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();
    
    octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", sensor_qos,
        std::bind(&SewerAutonomousTestNode::octomap_callback, this, std::placeholders::_1));
        
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", sensor_qos,
        std::bind(&SewerAutonomousTestNode::odometry_callback, this, std::placeholders::_1));
        
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    command_timer_ = this->create_wall_timer(
        5s, std::bind(&SewerAutonomousTestNode::command_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Sewer Autonomous Test Node initialized for VERTICAL exploration.");
}

void SewerAutonomousTestNode::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string previous_status = current_status_;
    current_status_ = msg->data;
    
    if (previous_status != current_status_) {
        if (current_status_ == "IDLE" && !last_command_sent_.empty()) {
            if (consecutive_failures_ > 0) {
                RCLCPP_INFO(this->get_logger(), "Command '%s' completed successfully. Resetting failure counter.", 
                           last_command_sent_.c_str());
                consecutive_failures_ = 0;
            }
        }
        else if (current_status_.find("ERROR") != std::string::npos || 
                 current_status_.find("FAILED") != std::string::npos) {
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), "Command '%s' failed with status '%s'. Consecutive failures: %d/%d", 
                       last_command_sent_.c_str(), current_status_.c_str(), 
                       consecutive_failures_, max_consecutive_failures_);
        }
    }
}

void SewerAutonomousTestNode::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
    if (tree) {
        octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
    }
    
    if (octree_) {
        static bool first_time = true;
        if (first_time || octomap_frame_id_ != msg->header.frame_id) {
            octomap_frame_id_ = msg->header.frame_id;
            first_time = false;
        }
    }
}

void SewerAutonomousTestNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pos_(0) = msg->pose.pose.position.x;
    current_pos_(1) = msg->pose.pose.position.y;
    current_pos_(2) = msg->pose.pose.position.z;
    has_odometry_ = true;
}

std::optional<Eigen::Vector3d> SewerAutonomousTestNode::find_vertical_frontier_goal(bool& should_return)
{
    should_return = false;
    if (!octree_) return std::nullopt;
    if (octree_->size() == 0) return std::nullopt;
    if (!has_odometry_) return std::nullopt;

    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
    } catch (const tf2::TransformException & ex) {
        return std::nullopt;
    }
    
    std::string target_frame = octomap_frame_id_.empty() ? "map" : octomap_frame_id_;
    
    geometry_msgs::msg::PoseStamped base_in_map;
    base_in_map.header.frame_id = "map";
    base_in_map.pose.position.x = map_to_base.transform.translation.x;
    base_in_map.pose.position.y = map_to_base.transform.translation.y;
    base_in_map.pose.position.z = map_to_base.transform.translation.z;
    base_in_map.pose.orientation = map_to_base.transform.rotation;
    
    geometry_msgs::msg::PoseStamped base_in_octomap;
    try {
        base_in_octomap = tf_buffer_->transform(base_in_map, target_frame, tf2::durationFromSec(0.1));
    } catch (tf2::TransformException &ex) {
        return std::nullopt;
    }

    double drone_x = base_in_octomap.pose.position.x;
    double drone_y = base_in_octomap.pose.position.y;
    double drone_z = base_in_octomap.pose.position.z;

    double max_descent = -1.0;
    bool found = false;
    
    // Pass 1: Find maximum free depth (-Z)
    for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();
            
            double lateral_dist = std::sqrt(std::pow(x - drone_x, 2) + std::pow(y - drone_y, 2));
            double descent_dist = drone_z - z; // positive means below drone
            
            if (descent_dist > 0 && descent_dist <= max_descent_distance_ && lateral_dist <= tunnel_radius_) {
                if (!found || descent_dist > max_descent) {
                    max_descent = descent_dist;
                    found = true;
                }
            }
        }
    }

    if (found && max_descent >= min_descent_space_) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        int count = 0;
        
        // Pass 2: Average X/Y for the deepest frontier to stay centered
        for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if (!octree_->isNodeOccupied(*it)) {
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();
                
                double lateral_dist = std::sqrt(std::pow(x - drone_x, 2) + std::pow(y - drone_y, 2));
                double descent_dist = drone_z - z;
                
                if (descent_dist >= max_descent - 1.0 && descent_dist <= max_descent && lateral_dist <= tunnel_radius_) {
                    sum_x += x;
                    sum_y += y;
                    count++;
                }
            }
        }
            
        if (count > 0) {
            double avg_x = sum_x / count;
            double avg_y = sum_y / count;
            
            // Move closer to the center of the detected free space, but dampen to avoid oscillation
            double goal_x = drone_x + (avg_x - drone_x) * 0.5;
            double goal_y = drone_y + (avg_y - drone_y) * 0.5;
            
            double target_descent = max_descent * goal_distance_ratio_;
            double goal_z = drone_z - target_descent;
            
            geometry_msgs::msg::PoseStamped goal_in_octomap;
            goal_in_octomap.header.frame_id = target_frame;
            goal_in_octomap.pose.position.x = goal_x;
            goal_in_octomap.pose.position.y = goal_y;
            goal_in_octomap.pose.position.z = goal_z;
            goal_in_octomap.pose.orientation.w = 1.0;
            
            geometry_msgs::msg::PoseStamped goal_in_map;
            try {
                goal_in_map = tf_buffer_->transform(goal_in_octomap, "map", tf2::durationFromSec(0.1));
                RCLCPP_INFO(this->get_logger(), "🎯 FOUND VERTICAL FRONTIER! Descending %.2fm (to map z=%.2f)", 
                            target_descent, goal_in_map.pose.position.z);
                return Eigen::Vector3d(goal_in_map.pose.position.x, goal_in_map.pose.position.y, goal_in_map.pose.position.z);
            } catch (tf2::TransformException &ex) {
                return std::nullopt;
            }
        }
    } else {
        if (!found || max_descent < min_descent_space_) {
            RCLCPP_WARN(this->get_logger(), "No sufficient downward space found! Max descent available: %.2f", max_descent);
            should_return = true;
        }
    }

    return std::nullopt;
}

void SewerAutonomousTestNode::send_explore_flyto()
{
    bool should_return = false;
    auto goal_opt = find_vertical_frontier_goal(should_return);
    
    if (should_return) {
        RCLCPP_WARN(this->get_logger(), "Bottom reached or blocked! Sending stop to hover.");
        // We could flyto(goal1) to return, but let's just stop or ascend.
        // The path_planner doesn't know "ascend". We can create a dynamic goal at Z=5.0
        std::string frame_name = "return_goal";
        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map"; 
        t.child_frame_id = frame_name;
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 5.0; 
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);
        
        send_command("flyto(" + frame_name + ")");
        return;
    }
    
    if (!goal_opt) return;
    
    Eigen::Vector3d goal = *goal_opt;
    exploration_goal_counter_++;
    std::string frame_name = "exploration_goal_" + std::to_string(exploration_goal_counter_);
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map"; 
    t.child_frame_id = frame_name;
    t.transform.translation.x = goal(0);
    t.transform.translation.y = goal(1);
    t.transform.translation.z = goal(2);
    
    // Keep current yaw
    try {
        auto map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
        t.transform.rotation = map_to_base.transform.rotation;
    } catch (...) {
        t.transform.rotation.w = 1.0;
    }
    
    tf_broadcaster_->sendTransform(t);
    send_command("flyto(" + frame_name + ")");
}

void SewerAutonomousTestNode::command_timer_callback()
{
    auto current_time = this->now();
    static auto last_command_time = current_time;
    static auto last_status_change = current_time;
    
    if (!system_initialized_) {
        if (is_system_idle()) {
            send_takeoff();
            system_initialized_ = true;
            last_command_time = current_time;
            last_status_change = current_time;
        }
        return;
    }
    
    static std::string previous_status = current_status_;
    if (current_status_ != previous_status) {
        last_status_change = current_time;
        previous_status = current_status_;
    }
    
    auto time_since_last_command = (current_time - last_command_time).seconds();
    auto time_since_status_change = (current_time - last_status_change).seconds();
    
    std::uniform_int_distribution<int> interval_dist(command_interval_min_, command_interval_max_);
    int target_interval = interval_dist(gen_);
    
    bool should_send_command = false;
    
    if (is_system_idle()) {
        if (time_since_last_command >= 5.0 && time_since_status_change >= 2.0) {
            should_send_command = true;
        }
    }
    else if (time_since_status_change >= max_wait_time_ && time_since_last_command >= target_interval) {
        should_send_command = true;
    }
    
    if (should_send_command) {
        if (consecutive_failures_ >= max_consecutive_failures_ && !last_command_was_land_ && last_command_sent_ != "land") {
            send_land();
            last_command_was_land_ = true;
            consecutive_failures_ = 0; 
        }
        else if (last_command_was_land_ || last_command_sent_ == "land") {
            send_takeoff();
            last_command_was_land_ = false;
        }
        else {
            double rand_val = static_cast<double>(command_dist_(gen_)) / 100.0;
            if (rand_val < land_probability_) {
                send_land();
                last_command_was_land_ = true;
            } else {
                send_explore_flyto();
                last_command_was_land_ = false;
            }
        }
        last_command_time = current_time;
        last_status_change = current_time;
    }
}

void SewerAutonomousTestNode::send_command(const std::string& command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    command_publisher_->publish(msg);
    last_command_sent_ = command;
    RCLCPP_INFO(this->get_logger(), "Sent command: '%s'", command.c_str());
}

void SewerAutonomousTestNode::send_takeoff()
{
    send_command("takeoff");
}

void SewerAutonomousTestNode::send_land()
{
    send_command("land");
}

bool SewerAutonomousTestNode::is_system_idle()
{
    return current_status_ == "IDLE" || 
           current_status_ == "STOPPED" || 
           current_status_ == "MISSION_COMPLETED";
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SewerAutonomousTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
