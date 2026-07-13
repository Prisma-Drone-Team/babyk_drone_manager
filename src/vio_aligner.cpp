#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>
#include <optional>

class VioAlignerNode : public rclcpp::Node
{
public:
    VioAlignerNode() : Node("vio_aligner_cpp")
    {
        // Static Transform Broadcaster
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        bool use_sim = false;
        // In ROS 2, use_sim_time is automatically populated if passed via launch files
        this->get_parameter("use_sim_time", use_sim);

        if (use_sim) {
            RCLCPP_INFO(this->get_logger(), "[SIMULATION] Waiting for Gazebo GT and VIO to calculate initial yaw offset...");
            
            sub_gt_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/model/baby_k_0/odometry", 10,
                std::bind(&VioAlignerNode::gt_cb, this, std::placeholders::_1));

            sub_vio_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ov_msckf/odomimu", 10,
                std::bind(&VioAlignerNode::vio_cb, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "[HARDWARE] Bypassing GT alignment. Forcing 0.0 offset and publishing TF immediately.");
            gt_yaw_ = 0.0;
            vio_yaw_ = 0.0;
            check_and_publish();
        }
    }

private:
    double euler_yaw_from_quaternion(double x, double y, double z, double w)
    {
        tf2::Quaternion q(x, y, z, w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void gt_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!gt_yaw_.has_value()) {
            auto q = msg->pose.pose.orientation;
            gt_yaw_ = euler_yaw_from_quaternion(q.x, q.y, q.z, q.w);
            check_and_publish();
        }
    }

    void vio_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!vio_yaw_.has_value()) {
            auto q = msg->pose.pose.orientation;
            vio_yaw_ = euler_yaw_from_quaternion(q.x, q.y, q.z, q.w);
            check_and_publish();
        }
    }

    void check_and_publish()
    {
        if (gt_yaw_.has_value() && vio_yaw_.has_value() && !published_) {
            published_ = true;

            // Gazebo ground truth from ros_gz_bridge is already in ENU (X=East, Y=North).
            // Drone facing East has yaw=0.
            double yaw_offset = gt_yaw_.value() - vio_yaw_.value();

            RCLCPP_INFO(this->get_logger(), "Ground Truth Yaw (ENU): %.3f rad", gt_yaw_.value());
            RCLCPP_INFO(this->get_logger(), "VIO Yaw: %.3f rad", vio_yaw_.value());
            RCLCPP_INFO(this->get_logger(), "Calculated Offset: %.3f rad", yaw_offset);
            RCLCPP_INFO(this->get_logger(), "Publishing aligned static TF: drone/map -> global and global -> odom");

            std::vector<geometry_msgs::msg::TransformStamped> transforms;
            rclcpp::Time now = this->get_clock()->now();

            // Transform 1: drone/map -> global
            geometry_msgs::msg::TransformStamped tf1;
            tf1.header.stamp = now;
            tf1.header.frame_id = "drone/map";
            tf1.child_frame_id = "global";
            tf1.transform.translation.x = 0.0;
            tf1.transform.translation.y = 0.0;
            tf1.transform.translation.z = 0.0;
            tf2::Quaternion q1;
            q1.setRPY(0, 0, yaw_offset);
            tf1.transform.rotation.x = q1.x();
            tf1.transform.rotation.y = q1.y();
            tf1.transform.rotation.z = q1.z();
            tf1.transform.rotation.w = q1.w();
            transforms.push_back(tf1);

            // Transform 2: global -> odom
            geometry_msgs::msg::TransformStamped tf2;
            tf2.header.stamp = now;
            tf2.header.frame_id = "global";
            tf2.child_frame_id = "odom";
            tf2.transform.translation.x = 0.0;
            tf2.transform.translation.y = 0.0;
            tf2.transform.translation.z = 0.0;
            tf2::Quaternion q2;
            // The global->odom transform must ALWAYS be -90 degrees (-M_PI_2).
            // This is required to align the VIO camera "Forward" (X) with the PX4 "North" (Y) in ENU.
            q2.setRPY(0, 0, -M_PI_2);
            tf2.transform.rotation.x = q2.x();
            tf2.transform.rotation.y = q2.y();
            tf2.transform.rotation.z = q2.z();
            tf2.transform.rotation.w = q2.w();
            transforms.push_back(tf2);

            // Publish static transforms
            static_tf_broadcaster_->sendTransform(transforms);
            
            // We can unsubscribe now that we've published the static transform
            sub_gt_.reset();
            sub_vio_.reset();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::optional<double> gt_yaw_;
    std::optional<double> vio_yaw_;
    bool published_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioAlignerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
