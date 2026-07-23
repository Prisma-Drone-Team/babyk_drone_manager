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
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            gt_q_ = q;
            vio_q_ = q;
            check_and_publish();
        }
    }

private:

    void gt_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!gt_q_.has_value()) {
            auto& o = msg->pose.pose.orientation;
            gt_q_ = tf2::Quaternion(o.x, o.y, o.z, o.w);
            check_and_publish();
        }
    }

    void vio_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!vio_q_.has_value()) {
            auto& o = msg->pose.pose.orientation;
            vio_q_ = tf2::Quaternion(o.x, o.y, o.z, o.w);
            check_and_publish();
        }
    }

    void check_and_publish()
    {
        if (gt_q_.has_value() && vio_q_.has_value() && !published_) {
            published_ = true;

            // Rotazione che porta VIO frame in GT frame:
            // q_offset = q_gt * q_vio^(-1)
            tf2::Quaternion q_offset = gt_q_.value() * vio_q_.value().inverse();
            q_offset.normalize();

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_offset).getRPY(roll, pitch, yaw);
            
            // THE HOLY GRAIL FIX: We MUST discard roll and pitch.
            // OpenVINS aligns its Z-axis perfectly with gravity.
            // If we apply Gazebo's physical roll/pitch tilt to the VIO pose, 
            // PX4's EKF2 will see a conflict between IMU gravity and Vision gravity!
            tf2::Quaternion q_yaw_only;
            q_yaw_only.setRPY(0.0, 0.0, yaw);
            q_yaw_only.normalize();
            
            RCLCPP_INFO(this->get_logger(), "Publishing aligned static TF: drone/map -> global and global -> odom");
            RCLCPP_INFO(this->get_logger(),
                "Offset calculated - roll: %.3f, pitch: %.3f, yaw: %.3f rad. APPLYING YAW ONLY!",
                roll, pitch, yaw);

            std::vector<geometry_msgs::msg::TransformStamped> transforms;
            rclcpp::Time now = this->get_clock()->now();

            // drone/map -> global: applica l'offset completo (yaw + pitch + roll)
            geometry_msgs::msg::TransformStamped tf1;
            tf1.header.stamp    = now;
            tf1.header.frame_id = "drone/map";
            tf1.child_frame_id  = "global";
            tf1.transform.translation.x = 0.0;
            tf1.transform.translation.y = 0.0;
            tf1.transform.translation.z = 0.0;
            tf1.transform.rotation.x = q_yaw_only.x();
            tf1.transform.rotation.y = q_yaw_only.y();
            tf1.transform.rotation.z = q_yaw_only.z();
            tf1.transform.rotation.w = q_yaw_only.w();
            transforms.push_back(tf1);

            // global -> odom: inverso dell'offset yaw-only
            tf2::Quaternion q_offset_inv = q_yaw_only.inverse();
            geometry_msgs::msg::TransformStamped tf2;
            tf2.header.stamp    = now;
            tf2.header.frame_id = "global";
            tf2.child_frame_id  = "odom";
            tf2.transform.translation.x = 0.0;
            tf2.transform.translation.y = 0.0;
            tf2.transform.translation.z = 0.0;
            tf2.transform.rotation.x = q_offset_inv.x();
            tf2.transform.rotation.y = q_offset_inv.y();
            tf2.transform.rotation.z = q_offset_inv.z();
            tf2.transform.rotation.w = q_offset_inv.w();
            transforms.push_back(tf2);

            static_tf_broadcaster_->sendTransform(transforms);
            
            // We can unsubscribe now that we've published the static transform
            sub_gt_.reset();
            sub_vio_.reset();
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gt_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vio_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    std::optional<tf2::Quaternion> gt_q_;
    std::optional<tf2::Quaternion> vio_q_;
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
