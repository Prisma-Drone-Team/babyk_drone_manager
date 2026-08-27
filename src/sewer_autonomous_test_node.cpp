#include "babyk_drone_manager/sewer_autonomous_test_node.h"
#include <sstream>
#include <iomanip>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

SewerAutonomousTestNode::SewerAutonomousTestNode()
    : Node("sewer_autonomous_test_node"),
      phase_(ExplorationPhase::INIT),
      current_status_("UNKNOWN"),
      consecutive_failures_(0),
      last_command_sent_(""),
      entry_command_sent_(false),
      has_odometry_(false),
      exploration_goal_counter_(0),
      gen_(rd_())
{
    this->declare_parameter("command_interval_min", 30);
    this->declare_parameter("command_interval_max", 45);
    this->declare_parameter("max_wait_time", 60);
    this->declare_parameter("max_consecutive_failures", 3);
    this->declare_parameter("goal_distance_ratio", 0.6);
    this->declare_parameter("min_descent_space", 0.5);
    this->declare_parameter("tunnel_radius", 2.0);
    this->declare_parameter("entry_frame", std::string("sewer_entry"));
    this->declare_parameter("entry_position_tolerance", 0.3);
    this->declare_parameter("reference_frame", std::string("drone/map"));
    this->declare_parameter("min_altitude_map", 1.0);
    this->declare_parameter("min_z_odom", -4.5);       // Z in odom ENU (Z=0 at spawn): -(spawn_z - floor_z - 0.5m)

    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_        = this->get_parameter("max_wait_time").as_int();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    goal_distance_ratio_  = this->get_parameter("goal_distance_ratio").as_double();
    min_descent_space_    = this->get_parameter("min_descent_space").as_double();
    tunnel_radius_        = this->get_parameter("tunnel_radius").as_double();
    entry_frame_               = this->get_parameter("entry_frame").as_string();
    entry_position_tolerance_  = this->get_parameter("entry_position_tolerance").as_double();
    reference_frame_           = this->get_parameter("reference_frame").as_string();
    min_altitude_map_          = this->get_parameter("min_altitude_map").as_double();
    min_z_odom_                = this->get_parameter("min_z_odom").as_double();

    command_publisher_ = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/command", 10);
    enable_fsm_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/fsm/enable_arming", 10);
    status_subscriber_ = this->create_subscription<std_msgs::msg::String>("/trajectory_interpolator/status", 10, std::bind(&SewerAutonomousTestNode::status_callback, this, std::placeholders::_1));

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();
    octomap_subscriber_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", sensor_qos, std::bind(&SewerAutonomousTestNode::octomap_callback, this, std::placeholders::_1));
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ov_msckf/odomimu", sensor_qos,
        std::bind(&SewerAutonomousTestNode::odometry_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    command_timer_ = this->create_wall_timer(5s, std::bind(&SewerAutonomousTestNode::command_timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Sewer Autonomous Test Node initialised.");
}

void SewerAutonomousTestNode::status_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string previous_status = current_status_;
    current_status_ = msg->data;
    if (previous_status != current_status_) {
        if (current_status_ == "IDLE" && !last_command_sent_.empty()) {
            if (consecutive_failures_ > 0) consecutive_failures_ = 0;
        } else if (current_status_.find("ERROR") != std::string::npos || current_status_.find("FAILED") != std::string::npos) {
            consecutive_failures_++;
        }
    }
}

void SewerAutonomousTestNode::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    if (!octree_) octree_ = std::make_shared<octomap::OcTree>(msg->resolution);
    octomap_frame_id_ = msg->header.frame_id;
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
    if (tree) {
        if (auto octree = dynamic_cast<octomap::OcTree*>(tree.get())) {
            octree_->clear();
            for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
                octree_->setNodeValue(it.getCoordinate(), it->getValue());
            }
        }
    }
}

void SewerAutonomousTestNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_(0) = msg->pose.pose.position.x;
    current_pos_(1) = msg->pose.pose.position.y;
    current_pos_(2) = msg->pose.pose.position.z;
    has_odometry_ = true;
}

std::optional<Eigen::Vector3d> SewerAutonomousTestNode::find_vertical_frontier_goal() {
    if (!octree_ || octree_->size() == 0 || !has_odometry_) return std::nullopt;
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
    } catch (...) { return std::nullopt; }

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
    } catch (...) { return std::nullopt; }

    double drone_x = base_in_octomap.pose.position.x;
    double drone_y = base_in_octomap.pose.position.y;
    double drone_z = base_in_octomap.pose.position.z;

    double max_descent = 0.0;
    bool found = false;
    for (octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
        if (!octree_->isNodeOccupied(*it)) {
            double descent_dist = drone_z - it.getZ();
            double lateral_dist = std::sqrt(std::pow(it.getX() - drone_x, 2) + std::pow(it.getY() - drone_y, 2));
            if (descent_dist > 0 && lateral_dist <= tunnel_radius_) {
                if (!found || descent_dist > max_descent) {
                    max_descent = descent_dist;
                    found = true;
                }
            }
        }
    }

    if (found && max_descent >= min_descent_space_) {
        double target_descent = max_descent * goal_distance_ratio_;
        double goal_z = drone_z - target_descent;
        
        geometry_msgs::msg::PoseStamped goal_in_octomap;
        goal_in_octomap.header.frame_id = target_frame;
        goal_in_octomap.pose.position.x = drone_x;
        goal_in_octomap.pose.position.y = drone_y;
        goal_in_octomap.pose.position.z = goal_z;
        goal_in_octomap.pose.orientation.w = 1.0;
        
        geometry_msgs::msg::PoseStamped goal_in_map;
        try {
            goal_in_map = tf_buffer_->transform(goal_in_octomap, "map", tf2::durationFromSec(0.1));
            return Eigen::Vector3d(goal_in_map.pose.position.x, goal_in_map.pose.position.y, goal_in_map.pose.position.z);
        } catch (...) { return std::nullopt; }
    }
    return std::nullopt;
}

void SewerAutonomousTestNode::send_explore_go() {
    // Safety floor check: direct Z comparison on PX4 odom ENU (Z=0 at spawn, negative going down).
    // Formula: -(spawn_z_gazebo - floor_z_gazebo - 0.5m) = -(5.8 - 0 - 0.5) = -5.3m theoretical.
    // Observed effective floor in simulation: ~-4.7m → default set to -4.5m (conservative).
    // Tune via 'min_z_odom' parameter in sewer_autonomous_test_node.yaml.
    if (has_odometry_ && current_pos_.z() < min_z_odom_) {
        RCLCPP_WARN(get_logger(),
            "[FLOOR CHECK] odom Z=%.2fm < soglia %.2fm → switch GOTO_ENTRY.",
            current_pos_.z(), min_z_odom_);
        phase_ = ExplorationPhase::GOTO_ENTRY;
        entry_command_sent_ = false;
        return;
    }

    auto goal_opt = find_vertical_frontier_goal();
    
    geometry_msgs::msg::PoseStamped target_map;
    target_map.header.frame_id = "map";
    target_map.pose.orientation.w = 1.0;
    
    if (goal_opt) {
        target_map.pose.position.x = (*goal_opt)(0);
        target_map.pose.position.y = (*goal_opt)(1);
        target_map.pose.position.z = (*goal_opt)(2);
    } else {
        RCLCPP_WARN(this->get_logger(), "Spazio libero inferiore a 0.5m o mappa ignota. Torno a sewer_entry!");
        send_goto_entry();
        return;
    }
    
    // Publish TF for visualization in RViz
    exploration_goal_counter_++;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = target_map.header.frame_id;
    t.child_frame_id = "exploration_goal_" + std::to_string(exploration_goal_counter_);
    t.transform.translation.x = target_map.pose.position.x;
    t.transform.translation.y = target_map.pose.position.y;
    t.transform.translation.z = target_map.pose.position.z;
    t.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(t);
    
    try {
        auto target_ref = tf_buffer_->transform(target_map, reference_frame_, tf2::durationFromSec(0.1));
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "go(" << target_ref.pose.position.x << "," << target_ref.pose.position.y << "," << target_ref.pose.position.z << ")";
        send_command(oss.str());
    } catch (...) {}
}

void SewerAutonomousTestNode::send_goto_entry() {
    if (entry_frame_.empty()) {
        phase_ = ExplorationPhase::EXPLORING;
        auto enable_msg = std_msgs::msg::Bool();
        enable_msg.data = true;
        enable_fsm_publisher_->publish(enable_msg);
        return;
    }
    try {
        geometry_msgs::msg::PoseStamped entry_map;
        entry_map.header.frame_id = entry_frame_;
        entry_map.pose.position.x = 0; entry_map.pose.position.y = 0; entry_map.pose.position.z = 0;
        entry_map.pose.orientation.w = 1.0;
        auto target_ref = tf_buffer_->transform(entry_map, reference_frame_, tf2::durationFromSec(0.1));
        
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3)
            << "go(" << target_ref.pose.position.x << "," << target_ref.pose.position.y << "," << target_ref.pose.position.z << ")";
        send_command(oss.str());
        entry_command_sent_ = true;
    } catch (...) {
        send_command("flyto(" + entry_frame_ + ")");
        entry_command_sent_ = true;
    }
}

void SewerAutonomousTestNode::command_timer_callback() {
    auto current_time = this->now();
    static auto last_command_time = current_time;
    static auto last_status_change = current_time;
    static std::string previous_status = current_status_;

    if (current_status_ != previous_status) {
        last_status_change = current_time;
        previous_status = current_status_;
    }

    if (phase_ == ExplorationPhase::INIT) {
        if (is_system_idle()) {
            send_takeoff();
            phase_ = ExplorationPhase::GOTO_ENTRY;
        }
        return;
    }

    if (phase_ == ExplorationPhase::GOTO_ENTRY) {
        if (is_system_idle() && current_status_ != "UNKNOWN") {
            if (!entry_command_sent_) {
                if (!tf_buffer_->canTransform("map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0))) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF tree (OpenVINS initialization) before sending sewer_entry goal...");
                    return;
                }
                send_goto_entry();
                last_command_time = current_time;
            } else {
                try {
                    auto map_to_base = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
                    auto map_to_entry = tf_buffer_->lookupTransform("map", entry_frame_, rclcpp::Time(0));
                    double dx = map_to_base.transform.translation.x - map_to_entry.transform.translation.x;
                    double dy = map_to_base.transform.translation.y - map_to_entry.transform.translation.y;
                    if (std::sqrt(dx*dx + dy*dy) <= entry_position_tolerance_) {
                        phase_ = ExplorationPhase::EXPLORING;
                        auto enable_msg = std_msgs::msg::Bool();
                        enable_msg.data = true;
                        enable_fsm_publisher_->publish(enable_msg);
                    } else if ((current_time - last_command_time).seconds() >= 10.0) {
                        send_goto_entry();
                        last_command_time = current_time;
                    }
                } catch (...) {}
            }
        }
        return;
    }

    if (phase_ == ExplorationPhase::EXPLORING) {
        if (is_system_idle() && (current_time - last_command_time).seconds() >= 2.0) {
            send_explore_go();
            last_command_time = current_time;
        }
    }
}

void SewerAutonomousTestNode::send_command(const std::string& command) {
    auto msg = std_msgs::msg::String();
    msg.data = command;
    command_publisher_->publish(msg);
    last_command_sent_ = command;
    RCLCPP_INFO(this->get_logger(), "Sent command: '%s'", command.c_str());
}

void SewerAutonomousTestNode::send_takeoff() { send_command("takeoff"); }
void SewerAutonomousTestNode::send_land() { send_command("land"); }
bool SewerAutonomousTestNode::is_system_idle() {
    return current_status_ == "IDLE" || current_status_ == "STOPPED" || current_status_ == "MISSION_COMPLETED";
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SewerAutonomousTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
