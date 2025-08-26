#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <mutex>

class MoveManagerNode : public rclcpp::Node {
public:
    MoveManagerNode();
    ~MoveManagerNode();

private:
    // ROS 2 publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_planner_goal_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_interp_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_active_pub_;  // NEW: teleop active flag

    // ROS 2 subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr path_planner_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traj_interp_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // TF2 for frame lookups
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // TF2 for simulation transforms
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Timer for TF publishing in simulation
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Parameters
    std::string command_topic_;
    std::string status_topic_;
    std::string path_planner_goal_topic_;
    std::string path_planner_status_topic_;
    std::string traj_interp_path_topic_;
    std::string traj_interp_status_topic_;
    std::string planned_path_topic_;
    std::string odometry_topic_;
    std::string joy_topic_;
    double takeoff_altitude_;
    double landing_altitude_;
    bool simulation_mode_;

    // State variables
    std::string current_command_;
    std::string received_command_;
    geometry_msgs::msg::Pose current_pose_;
    bool odometry_received_;
    std::atomic<bool> running_;
    std::string path_planner_status_;
    std::string traj_interp_status_;
    std::string overall_status_;
    bool teleop_active_;
    bool joy_available_;
    geometry_msgs::msg::Pose last_teleop_pose_;
    
    // Threading
    std::thread command_processor_thread_;
    std::mutex state_mutex_;

    // Callback functions
    void command_callback(const std_msgs::msg::String::SharedPtr msg);
    void path_planner_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void traj_interp_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void planned_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    // Command processing
    void command_processor();
    void process_command(const std::vector<std::string>& command_parts);
    
    // Command handlers
    void handle_flyto_command(const std::vector<std::string>& parts);
    void handle_go_command(const std::vector<std::string>& parts);
    void handle_takeoff_command(const std::vector<std::string>& parts);
    void handle_land_command(const std::vector<std::string>& parts);
    void handle_stop_command();
    void handle_teleop_command();

    // Utility functions
    std::vector<std::string> parse_command(const std::string& command);
    bool lookup_transform(const std::string& frame_name, geometry_msgs::msg::Pose& pose);
    nav_msgs::msg::Path create_direct_path(const geometry_msgs::msg::Pose& target_pose);
    void update_overall_status();
    void declare_parameters();
    
    // Simulation TF functions
    void timer_tf_callback();
    void static_tf_pub();
};
