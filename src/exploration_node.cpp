#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

class ExplorationNode : public rclcpp::Node {
public:
    ExplorationNode() : Node("exploration_node"), system_initialized_(false), is_moving_(false), odom_received_(false), octree_(nullptr) {
        // Parameters
        this->declare_parameter("min_x", -10.0);
        this->declare_parameter("max_x", 10.0);
        this->declare_parameter("min_y", -10.0);
        this->declare_parameter("max_y", 10.0);
        this->declare_parameter("min_z", 0.5);
        this->declare_parameter("max_z", 2.5);
        
        // Publishers & Subscribers
        command_pub_ = this->create_publisher<std_msgs::msg::String>("/seed_pdt_drone/command", 10);
        
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/move_manager/status", 10,
            std::bind(&ExplorationNode::status_callback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/baby_k_0/odometry", 10,
            std::bind(&ExplorationNode::odom_callback, this, std::placeholders::_1));
            
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", rclcpp::QoS(10).best_effort(),
            std::bind(&ExplorationNode::octomap_callback, this, std::placeholders::_1));
            
        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ExplorationNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Exploration Node initialized");
    }

    ~ExplorationNode() {
        if(octree_) delete octree_;
    }

private:
    bool system_initialized_;
    bool is_moving_;
    bool odom_received_;
    int wait_cycles_ = 0;
    std::string current_status_;
    
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_z_ = 0.0;
    
    octomap::OcTree* octree_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void status_callback(const std_msgs::msg::String::SharedPtr msg) {
        current_status_ = msg->data;
        if (current_status_ == "IDLE" || current_status_ == "STOPPED" || current_status_ == "MISSION_COMPLETED") {
            is_moving_ = false;
        } else {
            is_moving_ = true;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        odom_received_ = true;
    }

    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if(octree_) {
            delete octree_;
        }
        octomap::AbstractOcTree* tree = octomap_msgs::binaryMsgToMap(*msg);
        octree_ = dynamic_cast<octomap::OcTree*>(tree);
    }

    void timer_callback() {
        if (!odom_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry...");
            return;
        }

        // Wait a few cycles (e.g., 10 seconds) to let VIO and TF initialize fully
        if (wait_cycles_ < 5) {
            RCLCPP_INFO(this->get_logger(), "Waiting for VIO and TF to initialize... (%d/5)", wait_cycles_);
            wait_cycles_++;
            return;
        }

        // Step 1: Automatic Takeoff
        if (!system_initialized_) {
            if (current_status_ == "IDLE" || current_status_ == "STOPPED" || current_status_ == "ERROR_NO_ODOMETRY") {
                RCLCPP_INFO(this->get_logger(), "System ready and odometry received, sending automatic takeoff");
                std_msgs::msg::String cmd;
                cmd.data = "takeoff";
                command_pub_->publish(cmd);
                system_initialized_ = true;
                is_moving_ = true;
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for odometry and IDLE status...");
            }
            return;
        }

        // Step 2: Exploration
        if (!is_moving_ && system_initialized_ && octree_) {
            find_and_send_frontier();
        }
    }

    void find_and_send_frontier() {
        std::vector<octomap::point3d> frontiers;
        double min_x = this->get_parameter("min_x").as_double();
        double max_x = this->get_parameter("max_x").as_double();
        double min_y = this->get_parameter("min_y").as_double();
        double max_y = this->get_parameter("max_y").as_double();
        double min_z = this->get_parameter("min_z").as_double();
        double max_z = this->get_parameter("max_z").as_double();
        
        for(octomap::OcTree::leaf_iterator it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
            if(octree_->isNodeOccupied(*it)) continue; // We only care about free space
            
            octomap::point3d pt = it.getCoordinate();
            
            // Check bounding box
            if(pt.x() < min_x || pt.x() > max_x || pt.y() < min_y || pt.y() > max_y || pt.z() < min_z || pt.z() > max_z) {
                continue;
            }
            
            double res = octree_->getResolution();
            octomap::point3d dirs[6] = {
                octomap::point3d(res, 0, 0), octomap::point3d(-res, 0, 0),
                octomap::point3d(0, res, 0), octomap::point3d(0, -res, 0),
                octomap::point3d(0, 0, res), octomap::point3d(0, 0, -res)
            };
            
            bool is_frontier = false;
            for(int i = 0; i < 6; i++) {
                octomap::OcTreeNode* n_node = octree_->search(pt + dirs[i]);
                if(n_node == nullptr) { // Unknown space
                    is_frontier = true;
                    break;
                }
            }
            
            if(is_frontier) {
                frontiers.push_back(pt);
            }
        }
        
        if(frontiers.empty()) {
            RCLCPP_INFO(this->get_logger(), "No frontiers found in bounding box! Exploration might be complete.");
            return;
        }
        
        // Pick a good frontier: shuffle and pick one that's a bit away but not too far
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(frontiers.begin(), frontiers.end(), g);
        
        octomap::point3d current(current_x_, current_y_, current_z_);
        octomap::point3d best_frontier = frontiers.front();
        bool found = false;
        
        for(const auto& f : frontiers) {
            double dist = current.distance(f);
            if(dist > 1.5 && dist < 6.0) { // Avoid getting stuck in tiny movements
                best_frontier = f;
                found = true;
                break;
            }
        }
        
        std_msgs::msg::String cmd;
        char buf[100];
        snprintf(buf, sizeof(buf), "flyto(%.2f,%.2f,%.2f)", best_frontier.x(), best_frontier.y(), best_frontier.z());
        cmd.data = buf;
        command_pub_->publish(cmd);
        
        is_moving_ = true;
        RCLCPP_INFO(this->get_logger(), "🚀 Sent exploration goal to frontier: %s", cmd.data.c_str());
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
