#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/actuator_servos.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>  // per leggere l'odometria

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OpenBox : public rclcpp::Node
{
public:
    OpenBox() : Node("open_box")
    {
        n_sec_ = declare_parameter<double>("n_sec", 2.0);

        offboard_pub_ = create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        traj_pub_ = create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        cmd_pub_ = create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        servo_pub_ = create_publisher<ActuatorServos>(
            "/fmu/in/actuator_servos", 10);

        // Sottoscrizione all'odometria del drone (deve essere best_effort per i topic PX4)
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        odom_sub_ = create_subscription<VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&OpenBox::odom_callback, this, std::placeholders::_1));

        offboard_thread_ = std::thread(&OpenBox::offboard_loop, this);

        RCLCPP_INFO(get_logger(), "READY: in attesa della prima odometria... digita y per attivare il servo. n_sec=%.2f", n_sec_);
    }

    ~OpenBox() override
    {
        running_ = false;

        if (offboard_thread_.joinable()) {
            offboard_thread_.join();
        }
    }

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<ActuatorServos>::SharedPtr servo_pub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;

    std::thread offboard_thread_;
    std::atomic<bool> running_{true};
    std::atomic<bool> initial_pose_received_{false};

    // Posizione e yaw iniziali (impostate dalla prima odometria valida)
    float init_x_ = 0.0f;
    float init_y_ = 0.0f;
    float init_z_ = 0.0f;
    float init_yaw_ = 0.0f;
    std::mutex pose_mutex_;  // per accesso thread-safe alle variabili (anche se atomic basta per flag)

    int counter_ = 0;
    double n_sec_{2.0};

    // ---------------- CALLBACK ODOMETRIA ----------------
    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        // Salva la prima odometria valida
        if (!initial_pose_received_.load()) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            init_x_ = msg->position[0];
            init_y_ = msg->position[1];
            init_z_ = msg->position[2];

            // Estrai yaw dal quaternione (msg->q[0..3])
            // Usiamo la formula: yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            float qw = msg->q[0];
            float qx = msg->q[1];
            float qy = msg->q[2];
            float qz = msg->q[3];
            init_yaw_ = std::atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));

            initial_pose_received_.store(true);
            RCLCPP_INFO(get_logger(), "Odometria acquisita: pos (%.2f, %.2f, %.2f), yaw %.2f rad",
                        init_x_, init_y_, init_z_, init_yaw_);
        }
    }

    // ---------------- OFFBOARD THREAD ----------------
    void offboard_loop()
    {
        // Attendi finché non arriva la prima odometria
        while (running_ && rclcpp::ok() && !initial_pose_received_.load()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "In attesa di odometria...");
            std::this_thread::sleep_for(100ms);
        }

        if (!running_ || !rclcpp::ok()) return;

        RCLCPP_INFO(get_logger(), "Odometria ricevuta, avvio loop offboard");

        while (running_ && rclcpp::ok()) {
            publish_offboard();
            publish_setpoint();

            if (counter_ == 10) {
                arm();
                set_offboard();
            }

            counter_++;
            std::this_thread::sleep_for(100ms);
        }
    }

    // ---------------- OFFBOARD ----------------
    void publish_offboard()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.timestamp = now();
        offboard_pub_->publish(msg);
    }

    void publish_setpoint()
    {
        TrajectorySetpoint sp{};
        // Usa la posizione iniziale acquisita dall'odometria
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            sp.position = {init_x_, init_y_, init_z_};
            sp.yaw = init_yaw_;
        }
        sp.velocity = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        sp.jerk = {NAN, NAN, NAN};
        sp.yawspeed = NAN;
        
        sp.timestamp = now();
        traj_pub_->publish(sp);
    }

    void set_offboard()
    {
        send_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        RCLCPP_INFO(get_logger(), "OFFBOARD enabled");
    }

    void arm()
    {
        send_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        RCLCPP_INFO(get_logger(), "ARMED");
    }

    // ---------------- PERIPHERAL CMD (187) ----------------
    void send_peripheral(float v1)
    {
        VehicleCommand cmd{};
        cmd.command = VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR;
        cmd.param1 = v1;
        cmd.param2 = 0.f;
        cmd.param3 = 0.f;
        cmd.param4 = 0.f;
        cmd.param5 = 0.0;
        cmd.param6 = 0.0;
        cmd.param7 = 0.0f;  // Actuator Set 1
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.from_external = true;
        cmd.timestamp = now();
        cmd_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "PERIPHERAL CMD (187) sent to Set 1: %.2f", v1);
    }

    // ---------------- SERVO AUX1 ----------------
    void send_servo(float value)
    {
        ActuatorServos msg{};
        msg.timestamp = now();
        msg.timestamp_sample = now();
        for (int i = 0; i < 8; i++) {
            msg.control[i] = NAN;
        }
        msg.control[0] = value; // AUX1
        servo_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "SERVO AUX1: %.2f", value);
    }

    // ---------------- KEYS ----------------
    void handle_keys()
    {
        char c = 0;
        if (std::scanf(" %c", &c) != 1) {
            RCLCPP_ERROR(get_logger(), "Errore nella lettura del comando, chiudo il nodo");
            running_ = false;
            rclcpp::shutdown();
            return;
        }

        if (c == 'y') {
            send_peripheral(-1.0f);
            send_servo(-1.0f);

            RCLCPP_INFO(get_logger(), "Comando y ricevuto: +1 inviato, attendo %.2f s", n_sec_);
            std::this_thread::sleep_for(std::chrono::duration<double>(n_sec_));

            if (!running_ || !rclcpp::ok()) {
                return;
            }

            send_peripheral(1.0f);
            send_servo(1.0f);
            RCLCPP_INFO(get_logger(), "Ripubblicato -1, pronto per il prossimo comando");
            return;
        }

        RCLCPP_INFO(get_logger(), "Comando '%c' non valido, chiudo il nodo", c);
        running_ = false;
        rclcpp::shutdown();
    }

    void run_command_loop()
    {
        while (running_ && rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Attendo comando: y per attivare il servo");
            handle_keys();
        }
    }

    // ---------------- SEND CMD ----------------
    void send_cmd(uint16_t cmd, float p1, float p2 = 0.f)
    {
        VehicleCommand msg{};
        msg.command = cmd;
        msg.param1 = p1;
        msg.param2 = p2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.from_external = true;
        msg.timestamp = now();
        cmd_pub_->publish(msg);
    }

    uint64_t now()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

public:
    void spin_input_loop()
    {
        run_command_loop();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OpenBox>();
    
    // Eseguiamo rclcpp::spin in un thread separato così ROS2 può ricevere i messaggi
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });
    
    // Il thread principale legge i comandi da tastiera (bloccante)
    node->spin_input_loop();
    
    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    return 0;
}
