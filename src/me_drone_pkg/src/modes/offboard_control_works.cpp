#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node")
    {
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        timer_ = create_wall_timer(50ms, std::bind(&OffboardControl::tick, this));
    }

private:
    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }

    void tick()
    {
        uint64_t ts = now_us();
        count_++;

        // Publicar SIEMPRE los tres topics en cada ciclo
        publish_offboard_control_mode(ts);
        publish_trajectory_setpoint(ts);

        // Cambio de modo y armado tras 0.5 s de streaming (count==10 a 20 Hz)
        if (count_ == 10) set_offboard_mode(ts);
        if (count_ == 25) arm(ts);
    }

    void publish_offboard_control_mode(uint64_t ts)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp  = ts;
        msg.position   = true;
        msg.velocity   = false;
        msg.acceleration = false;
        msg.attitude   = false;
        msg.body_rate  = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(uint64_t ts)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;
        sp.position     = {0.0f, 0.0f, -5.0f};
        sp.acceleration  = {NAN, NAN, NAN};
        sp.yaw           = 0.0f;

     
        trajectory_setpoint_pub_->publish(sp);
    }

    void arm(uint64_t ts)
    {
        send_vehicle_command(ts, 400, 1.0f);
        RCLCPP_INFO(get_logger(), "Armando...");
    }

    void set_offboard_mode(uint64_t ts)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp        = ts;
        msg.command          = 176;   // VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0f;  // custom mode
        msg.param2           = 6.0f;  // OFFBOARD
        msg.param3           = 0.0f;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Cambiando a modo OFFBOARD...");
    }

    void send_vehicle_command(uint64_t ts, uint16_t command, float param1)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp        = ts;
        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = 0.0f;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        vehicle_command_pub_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    int count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}