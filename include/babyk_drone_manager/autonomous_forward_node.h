#ifndef AUTONOMOUS_FORWARD_NODE_H
#define AUTONOMOUS_FORWARD_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <random>
#include <cmath>

class AutonomousForwardNode : public rclcpp::Node
{
public:
    AutonomousForwardNode();

private:
    void command_timer_callback();
    void status_callback(const std_msgs::msg::String::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void send_command(const std::string& command);
    void send_takeoff();
    void send_land();
    void send_forward_flyto();
    bool is_system_idle();

    // ROS2 interfaces
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr takeoff_completed_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    // State
    std::string current_status_{"UNKNOWN"};
    bool last_command_was_land_{false};
    bool system_initialized_{false};
    bool takeoff_completed_announced_{false};
    bool has_odometry_{false};
    int consecutive_failures_{0};
    std::string last_command_sent_;
    int goal_counter_{0};

    // Map frame: published once after 10 odometry messages, 1m ahead of the drone
    bool map_frame_published_{false};
    int odometry_msg_count_{0};

    // Odometry
    double pos_x_{0}, pos_y_{0}, pos_z_{0};
    double yaw_{0};

    // Random
    std::random_device rd_;
    std::mt19937 gen_;

    // Parameters
    double forward_distance_;       // How far ahead to place goals (meters)
    double goal_height_;            // Fixed flight height (meters)
    double lateral_variation_;      // Random lateral offset (meters)
    int command_interval_min_;
    int command_interval_max_;
    int max_wait_time_;
    double land_probability_;
    int max_consecutive_failures_;
    std::string parent_frame_;      // TF parent frame for goal frames (must match move_manager)
};

#endif
