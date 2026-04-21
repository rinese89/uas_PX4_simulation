#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <string>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node")
    {
        // =========================
        // PARÁMETROS
        // =========================
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<int>("target_system", 1);
        this->declare_parameter<double>("target_alt", 5.0);

        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("target_system", target_system_);
        this->get_parameter("target_alt", target_alt_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        RCLCPP_INFO(this->get_logger(), "Namespace: %s", ns_prefix_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target system: %d", target_system_);
        RCLCPP_INFO(this->get_logger(), "Target altitude: %.2f m", target_alt_);

        // =========================
        // PUBS
        // =========================
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            ns_prefix_ + "/fmu/in/vehicle_command", 10);

        ocm_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            ns_prefix_ + "/fmu/in/offboard_control_mode", 10);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            ns_prefix_ + "/fmu/in/trajectory_setpoint", 10);

        // =========================
        // SUBS
        // =========================
        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.best_effort();

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            ns_prefix_ + "/fmu/out/vehicle_odometry", qos,
            std::bind(&OffboardControl::odom_callback, this, std::placeholders::_1));

        // =========================
        // SERVICIO DE CESIÓN DE CONTROL
        // =========================
        control_srv_ = this->create_service<std_srvs::srv::Trigger>(
            ns_prefix_ + "/enable_control",
            std::bind(&OffboardControl::control_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // =========================
        // CLIENTE DE SOLICITUD DE VUELO
        // =========================
        authorit_client_ = create_client<std_srvs::srv::Trigger>(
            ns_prefix + "/enable_fly_authorization");

        // =========================
        // TIMER
        // =========================
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControl::tick, this));

        RCLCPP_INFO(this->get_logger(), "Offboard control iniciado (ns=%s).",
                    drone_ns_.c_str());
    }

private:
    std::string drone_ns_;
    std::string ns_prefix_;
    int target_system_{1};
    double target_alt_{5.0};

    std::string state_{"takeoff"};
    float current_alt_{0.0f};
    int count_{0};

    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    // =========================
    // ODOM
    // =========================
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_alt_ = -msg->position[2];
    }


    // =========================
    // VEHICLE COMMAND
    // =========================
    void send_vehicle_command(uint16_t cmd, float p1, float p2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp        = now_us();
        msg.command          = cmd;
        msg.param1           = p1;
        msg.param2           = p2;
        msg.target_system    = target_system_;
        msg.target_component = 1;
        msg.source_system    = target_system_;
        msg.source_component = 1;
        msg.from_external    = true;

        vehicle_command_pub_->publish(msg);
    }

    // =========================
    // LOOP
    // =========================
    void tick()
    {
        const uint64_t ts = now_us();
        ++count_;

        if (state_ == "takeoff") {
            publish_offboard_control_mode(ts);
            publish_takeoff_setpoint(ts);
            publish_state("takeoff");

            if (count_ == 50) {
                // VEHICLE_CMD_DO_SET_MODE = 176
                // param1 = 1.0 (custom mode enabled)
                // param2 = 6.0 (offboard)
                send_vehicle_command(176, 1.0f, 6.0f);
            }

            if (count_ == 100) {
                // VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
                send_vehicle_command(400, 1.0f);
            }
        }
        else if (state_ == "control") {
            // Aquí no publicamos nada.
            // El nodo externo que tomó el control debe publicar
            // OffboardControlMode y TrajectorySetpoint.
            publish_state("control");
        }

        if (count_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(),
                "[%s] Alt: %.2f m | Estado: %s",
                drone_ns_.c_str(), current_alt_, state_.c_str());
        }
    }

    // =========================
    // SERVICIO CONTROL
    // =========================
    void control_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        state_ = "control";
        publish_state("control");

        res->success = true;
        res->message = "Control cedido";

        RCLCPP_INFO(this->get_logger(), "[%s] Cambio a CONTROL", drone_ns_.c_str());
    }

    // =========================
    // OFFBOARD
    // =========================
    void publish_offboard_control_mode(uint64_t ts)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp    = ts;
        msg.position     = true;
        msg.velocity     = false;
        msg.acceleration = false;
        msg.attitude     = false;
        msg.body_rate    = false;

        ocm_pub_->publish(msg);
    }

    void publish_takeoff_setpoint(uint64_t ts)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp    = ts;
        sp.position     = {
            0.0f,
            0.0f,
            static_cast<float>(-target_alt_)
        };
        sp.velocity     = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        sp.yaw          = 0.0f;
        sp.yawspeed     = NAN;

        setpoint_pub_->publish(sp);
    }

    // =========================
    // MEMBERS
    // =========================
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr control_srv_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}