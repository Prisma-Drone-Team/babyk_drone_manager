#include "babyk_drone_manager/autonomous_test_node.h"

using namespace std::chrono_literals;

AutonomousTestNode::AutonomousTestNode() 
    : Node("autonomous_test_node"),
      current_status_("UNKNOWN"),
      last_command_was_land_(false),
      system_initialized_(false),
      gen_(rd_()),
      command_dist_(0, 99)
{
    // Declare and get parameters
    this->declare_parameter("command_interval_min", 120);  // 2 minutes
    this->declare_parameter("command_interval_max", 180);  // 3 minutes
    this->declare_parameter("max_wait_time", 60);          // 1 minute max wait
    this->declare_parameter("land_probability", 0.20);     // 20% chance of landing
    
    command_interval_min_ = this->get_parameter("command_interval_min").as_int();
    command_interval_max_ = this->get_parameter("command_interval_max").as_int();
    max_wait_time_ = this->get_parameter("max_wait_time").as_int();
    land_probability_ = this->get_parameter("land_probability").as_double();
    
    // Available goals (goal1 to goal7)
    available_goals_ = {"goal1", "goal2", "goal3", "goal4", "goal5", "goal6", "goal7"};

    // Create publisher and subscriber
    command_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/move_manager/command", 10);
    
    status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/trajectory_interpolator/status", 10,
        std::bind(&AutonomousTestNode::status_callback, this, std::placeholders::_1));
    
    // Create timer for sending commands
    command_timer_ = this->create_wall_timer(
        5s, std::bind(&AutonomousTestNode::command_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Autonomous Test Node initialized");
    RCLCPP_INFO(this->get_logger(), "Available goals: goal1-goal7");
    RCLCPP_INFO(this->get_logger(), "Command interval: %d-%d seconds", 
                command_interval_min_, command_interval_max_);
    RCLCPP_INFO(this->get_logger(), "Land probability: %.1f%%", land_probability_ * 100);
}

void AutonomousTestNode::status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    current_status_ = msg->data;
    RCLCPP_DEBUG(this->get_logger(), "Status: %s", current_status_.c_str());
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
        // If system just became idle, send command immediately
        // Otherwise wait for the interval
        if (time_since_last_command >= 10.0) {  // Minimum 10 seconds between commands
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
        // If last command was land, next must be takeoff
        if (last_command_was_land_) {
            send_takeoff();
            last_command_was_land_ = false;
        }
        else {
            // Random choice between flyto and land
            double rand_val = static_cast<double>(command_dist_(gen_)) / 100.0;
            
            if (rand_val < land_probability_) {
                send_land();
                last_command_was_land_ = true;
            }
            else {
                send_random_flyto();
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
    RCLCPP_INFO(this->get_logger(), "Sent command: '%s'", command.c_str());
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
    // Pick a random goal from goal1 to goal7
    std::uniform_int_distribution<int> goal_dist(0, available_goals_.size() - 1);
    int goal_index = goal_dist(gen_);
    
    std::string command = "flyto(" + available_goals_[goal_index] + ")";
    send_command(command);
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