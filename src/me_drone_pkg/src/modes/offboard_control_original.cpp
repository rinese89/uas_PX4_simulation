#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node")
    {
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // 20 Hz recomendado
        timer_ = create_wall_timer(50ms, std::bind(&OffboardControl::tick, this));

        timestamp_ = now_us();
    }

private:
    uint64_t now_us() const { return this->get_clock()->now().nanoseconds() / 1000; }

    void tick()
    {
        timestamp_ = now_us();
        count_++;

        publish_offboard_control_mode(); // velocity=true, resto=false

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp =timestamp_;     // o now_us() si ya te funciona estable
        sp.position = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        if (count_ < 100) {
          // Despegue OFFBOARD durante ~5 s a -0.5 m/s
          sp.velocity = {0.0f, 0.0f, -0.5f}; // NED: z<0 es subir
          sp.yaw = NAN;                      // no controlar yaw
        } else {
          // Avance continuo
          sp.velocity = {1.0f, 0.0f, 0.0f};  // 1 m/s hacia el Norte local
          sp.yaw = NAN;                      // no imponer rumbo
        }
        trajectory_setpoint_pub_->publish(sp);

        // Cambios de modo/arm una sola vez, después de ~0.5 s de streaming:
        if (count_ == 10) set_offboard_mode(); // param1=1 (custom), param2=6 (OFFBOARD), param3=0
        if (count_ == 15) arm();
    }

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = timestamp_;
        // Control por VELOCIDAD en marco local (NED)
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),"Publishing OFF_BCM");

    }

    void publish_velocity_setpoint()
    {
        // Queremos "avanzar" respecto al yaw actual (aquí fijo como ejemplo).
        // Si ya conoces tu yaw deseado, úsalo; si no, usa 0 para Norte.
        const float yaw = 0.0f;           // rad (0 => proa al Norte)
        const float v_fwd = 1.0f;         // m/s

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = timestamp_;

        // Cuando controlas por VELOCIDAD, deja position/accel en NaN para no confundir al controlador
        msg.position = {std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()};
        msg.acceleration = {std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::quiet_NaN()};

        // Proyecta velocidad "hacia adelante" al marco local NED
        msg.velocity = {v_fwd * std::cos(yaw),  // X (Norte)
                        v_fwd * std::sin(yaw),  // Y (Este)
                        0.0f};                  // Z positivo es "abajo" en NED; 0 para mantener altura

        msg.yaw = yaw;       // opcional; rad
        // msg.yawspeed = 0; // opcional
        RCLCPP_INFO(this->get_logger(),"Publishing velocity");

        trajectory_setpoint_pub_->publish(msg);
    }

    void arm()
    {
        send_vehicle_command(400, 1.0f); // VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1 arm
        RCLCPP_INFO(get_logger(), "Armando (ARM)...");
    }

    void set_offboard_mode()
    {
        // VEHICLE_CMD_DO_SET_MODE (176)
        // param1 = 1 (custom mode enabled)
        // param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        // param3 = 0 (submodo)
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = timestamp_;
        msg.command = 176;
        msg.param1 = 1.0f;
        msg.param2 = 6.0f;
        msg.param3 = 0.0f;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Cambiando a OFFBOARD...");
    }

    void send_vehicle_command(uint16_t command, float param1)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = timestamp_;
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = 0.0f;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    uint64_t timestamp_{};
    int count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
