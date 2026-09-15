#include "babyk_drone_manager/autonomous_forward_node.h"

using namespace std::chrono_literals;

AutonomousForwardNode::AutonomousForwardNode()
    : Node("autonomous_forward_node"), gen_(rd_())
{
    // Parameters
    this->declare_parameter("forward_distance", 3.0);
    this->declare_parameter("goal_height", 1.5);
    this->declare_parameter("lateral_variation", 0.5);
    this->declare_parameter("command_interval_min", 30);
    this->declare_parameter("command_interval_max", 60);
    this->declare_parameter("max_wait_time", 60);
    this->declare_parameter("land_probability", 0.0);
    this->declare_parameter("max_consecutive_failures", 3);
    this->declare_parameter("parent_frame", std::string("drone/map"));

    forward_distance_ = this->get_parameter("forward_distance").as_double();
    goal_height_ = this->get_parameter("goal_height").as_double();
    lateral_variation_ = this->get_parameter("lateral_variation").as_double();
    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_ = this->get_parameter("max_wait_time").as_int();
    land_probability_ = this->get_parameter("land_probability").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    parent_frame_ = this->get_parameter("parent_frame").as_string();

    // Publishers & subscribers
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/seed_pdt_drone/command", 10);
        
    takeoff_completed_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/autonomous_node/takeoff_completed", 10);

    status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/trajectory_interpolator/status", 10,
        std::bind(&AutonomousForwardNode::status_callback, this, std::placeholders::_1));

    rclcpp::QoS sensor_qos(rclcpp::KeepLast(10));
    sensor_qos.best_effort();

    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/px4/odometry/out", sensor_qos,
        std::bind(&AutonomousForwardNode::odometry_callback, this, std::placeholders::_1));



    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    command_timer_ = this->create_wall_timer(
        5s, std::bind(&AutonomousForwardNode::command_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Autonomous Forward Node initialized");
    RCLCPP_INFO(this->get_logger(), "  forward_distance: %.1fm", forward_distance_);
    RCLCPP_INFO(this->get_logger(), "  goal_height: %.1fm", goal_height_);
    RCLCPP_INFO(this->get_logger(), "  lateral_variation: %.1fm", lateral_variation_);
    RCLCPP_INFO(this->get_logger(), "  command_interval: %d-%ds", command_interval_min_, command_interval_max_);
}

void AutonomousForwardNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    pos_z_ = msg->pose.pose.position.z;

    // Extract yaw from quaternion
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    yaw_ = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // Publish static TFs ONCE after receiving 20 odometry messages (to let VIO stabilize).
    // TF tree:
    //   map (ROOT, 1m ahead of drone)
    //    └── drone/map  (identity, from tf_static_flight)
    //         └── global  (offset = odom origin in drone/map)
    //              └── odom  (identity with global)
    //                   └── base_link  (dynamic)
    
    odometry_msg_count_++;
    
    if (!map_frame_published_ && odometry_msg_count_ > 20) {
        map_frame_published_ = true;

        // map origin in odom frame: 1m ahead of drone (identity orientation)
        double map_tx = pos_x_ + std::cos(yaw_);
        double map_ty = pos_y_ + std::sin(yaw_);
        double map_tz = pos_z_;

        // drone/map == map (identity), so global offset in drone/map = -map origin
        double global_in_dm_x = -map_tx;
        double global_in_dm_y = -map_ty;
        double global_in_dm_z = -map_tz;

        RCLCPP_INFO(this->get_logger(), "🗺️ Mapping initialized! global_in_dm offset = [X: %.3f, Y: %.3f, Z: %.3f]", 
                    global_in_dm_x, global_in_dm_y, global_in_dm_z);

        // Build drone/map → global static TF (identity rotation)
        geometry_msgs::msg::TransformStamped dm_to_global;
        dm_to_global.header.stamp    = this->get_clock()->now();
        dm_to_global.header.frame_id = "drone/map";
        dm_to_global.child_frame_id  = "global";
        dm_to_global.transform.translation.x = global_in_dm_x;
        dm_to_global.transform.translation.y = global_in_dm_y;
        dm_to_global.transform.translation.z = 0.0;
        dm_to_global.transform.rotation.x = 0.0;
        dm_to_global.transform.rotation.y = 0.0;
        dm_to_global.transform.rotation.z = 0.0;
        dm_to_global.transform.rotation.w = 1.0;

        // Build global → odom static TF (identity: odom coincident with global)
        geometry_msgs::msg::TransformStamped global_to_odom;
        global_to_odom.header.stamp    = this->get_clock()->now();
        global_to_odom.header.frame_id = "global";
        global_to_odom.child_frame_id  = "odom";
        global_to_odom.transform.translation.x = 0.0;
        global_to_odom.transform.translation.y = 0.0;
        global_to_odom.transform.translation.z = 0.0;
        global_to_odom.transform.rotation.x = 0.0;
        global_to_odom.transform.rotation.y = 0.0;
        global_to_odom.transform.rotation.z = 0.0;
        global_to_odom.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform({dm_to_global, global_to_odom});

        RCLCPP_INFO(this->get_logger(),
            "🗺️  TF tree published — map ROOT 1m ahead of drone:\n"
            "   map origin in odom: [%.2f, %.2f, %.2f]\n"
            "   global in drone/map: [%.2f, %.2f, %.2f]\n"
            "   odom: identity child of global",
            map_tx, map_ty, map_tz,
            global_in_dm_x, global_in_dm_y, global_in_dm_z);
    }

    has_odometry_ = true;
}



void AutonomousForwardNode::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string previous = current_status_;
    current_status_ = msg->data;

    if (previous != current_status_) {
        if (current_status_ == "IDLE" && !last_command_sent_.empty()) {
            if (consecutive_failures_ > 0) {
                RCLCPP_INFO(this->get_logger(), "Command completed. Resetting failure counter.");
                consecutive_failures_ = 0;
            }
        } else if (current_status_.find("ERROR") != std::string::npos ||
                   current_status_.find("FAILED") != std::string::npos) {
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), "Command failed: '%s'. Failures: %d/%d",
                        current_status_.c_str(), consecutive_failures_, max_consecutive_failures_);
        }
    }
}


void AutonomousForwardNode::send_forward_flyto()
{
    if (!has_odometry_) {
        RCLCPP_WARN(this->get_logger(), "No odometry yet, cannot generate goal.");
        return;
    }

    // Generate goal: forward_distance_ meters ahead in the drone's heading direction
    // with optional lateral variation
    std::uniform_real_distribution<double> lat_dist(-lateral_variation_, lateral_variation_);
    double lateral_offset = lat_dist(gen_);

    // Forward direction from yaw (in ENU: yaw=0 → East, yaw=π/2 → North)
    double fwd_x = std::cos(yaw_);
    double fwd_y = std::sin(yaw_);
    // Lateral (left) direction
    double lat_x = -fwd_y;
    double lat_y = fwd_x;

    double goal_x = pos_x_ + forward_distance_ * fwd_x + lateral_offset * lat_x;
    double goal_y = pos_y_ + forward_distance_ * fwd_y + lateral_offset * lat_y;
    double goal_z = goal_height_;

    // Publish TF frame for the goal — child of map, identity orientation
    goal_counter_++;
    std::string frame_name = "fwd_goal_" + std::to_string(goal_counter_);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";  // goals are direct children of map
    t.child_frame_id = frame_name;
    t.transform.translation.x = goal_x;
    t.transform.translation.y = goal_y;
    t.transform.translation.z = goal_z;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;  // identity = same orientation as map
    tf_broadcaster_->sendTransform(t);

    // Send flyto command
    std::string command = "flyto(" + frame_name + ")";
    send_command(command);

    RCLCPP_INFO(this->get_logger(),
        "🚀 Goal #%d: [%.2f, %.2f, %.2f] (%.1fm ahead, lat=%.2f, orientation=map)",
        goal_counter_, goal_x, goal_y, goal_z,
        forward_distance_, lateral_offset);
}

void AutonomousForwardNode::command_timer_callback()
{
    static auto last_command_time = this->now();
    static auto last_status_change = this->now();
    static std::string previous_status = current_status_;

    auto current_time = this->now();

    // --- Phase 1: Initialization (runs once) → automatic takeoff ---
    if (!system_initialized_) {
        if (is_system_idle()) {
            RCLCPP_INFO(this->get_logger(), "🚀 System idle → sending automatic takeoff");
            send_takeoff();
            system_initialized_ = true;
            last_command_time  = current_time;
            last_status_change = current_time;
        }
        return;
    }

    if (current_status_ != previous_status) {
        last_status_change = current_time;
        previous_status = current_status_;
    }

    auto time_since_cmd = (current_time - last_command_time).seconds();
    auto time_since_status = (current_time - last_status_change).seconds();

    std::uniform_int_distribution<int> interval_dist(command_interval_min_, command_interval_max_);
    int target_interval = interval_dist(gen_);

    bool should_send = false;

    // Publish takeoff status continuously so late subscribers don't miss it
    auto takeoff_msg = std_msgs::msg::Bool();
    takeoff_msg.data = takeoff_completed_announced_;
    takeoff_completed_pub_->publish(takeoff_msg);

    if (is_system_idle()) {
        if (!takeoff_completed_announced_ && system_initialized_ && last_command_sent_ == "takeoff" && time_since_cmd >= 10.0) {
            takeoff_completed_announced_ = true;
            RCLCPP_INFO(this->get_logger(), "📣 Takeoff completed announced!");
        }

        if (time_since_cmd >= 10.0 && time_since_status >= 2.0) {
            should_send = true;
        }
    } else if (time_since_status >= max_wait_time_ && time_since_cmd >= target_interval) {
        should_send = true;
        RCLCPP_WARN(this->get_logger(), "System stuck in '%s' for %.1fs, forcing command",
                    current_status_.c_str(), time_since_status);
    }

    if (should_send) {
        // Emergency land
        if (consecutive_failures_ >= max_consecutive_failures_ && !last_command_was_land_) {
            RCLCPP_ERROR(this->get_logger(), "🚨 EMERGENCY LAND: %d consecutive failures!", consecutive_failures_);
            send_land();
            last_command_was_land_ = true;
            consecutive_failures_ = 0;
        }
        // Map limit reached
        else if (pos_x_ >= 2.75) {
            if (!last_command_was_land_) {
                RCLCPP_INFO(this->get_logger(), "🏁 Reached map limit (x close to 3.0 m). Sending LAND command.");
                send_land();
                last_command_was_land_ = true;
            } else {
                RCLCPP_INFO(this->get_logger(), "🏁 Mission complete at map limit. Staying on the ground.");
            }
        }
        // After land → mandatory takeoff
        else if (last_command_was_land_ || last_command_sent_ == "land") {
            send_takeoff();
            last_command_was_land_ = false;
            takeoff_completed_announced_ = false; // Reset for next takeoff
            RCLCPP_INFO(this->get_logger(), "MANDATORY takeoff after land");
        }
        else {
            std::uniform_real_distribution<double> prob(0.0, 1.0);
            if (prob(gen_) < land_probability_) {
                send_land();
                last_command_was_land_ = true;
            } else {
                send_forward_flyto();
                last_command_was_land_ = false;
            }
        }

        last_command_time = current_time;
        last_status_change = current_time;
    }
}

void AutonomousForwardNode::send_command(const std::string& command)
{
    auto msg = std_msgs::msg::String();
    msg.data = command;
    command_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "✅ Sent: '%s'", command.c_str());
    last_command_sent_ = command;
}

void AutonomousForwardNode::send_takeoff()
{
    send_command("takeoff");
    RCLCPP_INFO(this->get_logger(), "🚀 Sent takeoff command! Drone is currently at: [%.2f, %.2f, %.2f]", pos_x_, pos_y_, pos_z_);
}
void AutonomousForwardNode::send_land() { send_command("land"); }

bool AutonomousForwardNode::is_system_idle()
{
    return current_status_ == "IDLE" ||
           current_status_ == "STOPPED" ||
           current_status_ == "TRAJECTORY_COMPLETED" ||
           current_status_ == "DIRECT_PATH_COMPLETED" ||
           current_status_ == "MISSION_COMPLETED";
}



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousForwardNode>());
    rclcpp::shutdown();
    return 0;
}
