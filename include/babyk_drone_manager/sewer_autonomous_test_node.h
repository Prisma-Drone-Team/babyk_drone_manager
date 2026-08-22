#ifndef SEWER_AUTONOMOUS_TEST_NODE_H
#define SEWER_AUTONOMOUS_TEST_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <optional>

enum class ExplorationPhase {
    INIT,
    GOTO_ENTRY,
    EXPLORING
};

class SewerAutonomousTestNode : public rclcpp::Node
{
public:
    SewerAutonomousTestNode();

private:
    void command_timer_callback();
    void status_callback(const std_msgs::msg::String::SharedPtr msg);
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void send_command(const std::string& command);
    void send_takeoff();
    void send_land();
    void send_goto_entry();
    void send_explore_go();
    bool is_system_idle();

    std::optional<Eigen::Vector3d> find_vertical_frontier_goal();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_fsm_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscriber_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::TimerBase::SharedPtr command_timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    ExplorationPhase phase_;

    int command_interval_min_;
    int command_interval_max_;
    int max_wait_time_;
    int max_consecutive_failures_;
    double goal_distance_ratio_;
    double min_descent_space_;
    double tunnel_radius_;
    std::string entry_frame_;
    double entry_position_tolerance_;
    std::string reference_frame_;

    std::string current_status_;
    int consecutive_failures_;
    std::string last_command_sent_;
    bool entry_command_sent_;

    std::shared_ptr<octomap::OcTree> octree_;
    std::string octomap_frame_id_ = "";
    Eigen::Vector3d current_pos_;
    bool has_odometry_;
    int exploration_goal_counter_;

    std::random_device rd_;
    std::mt19937 gen_;
};

#endif
