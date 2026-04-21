#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node")
    {
        // Publishers
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // Timer para publicar mensajes
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControl::publish_messages, this));

        // Inicialización
        timestamp_ = this->get_clock()->now().nanoseconds() / 1000;  // en microsegundos
        count_ = 0;
         
    }

private:
    void publish_messages()
{
    timestamp_ = this->get_clock()->now().nanoseconds() / 1000;

    if (count_ == 10) {
        arm();
    }

    if (count_ == 30) {
        takeoff(5.0f);  // Altitud deseada en metros
    }

    if (count_ == 80) {
        set_offboard_mode();
        init = true;
    }

    if (init) {
        publish_offboard_control_mode();
        publish_trajectory_setpoint();
    }

    count_++;
}

    void publish_offboard_control_mode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = timestamp_;
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint()
    {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = timestamp_;
        msg.velocity = {1.0f,0.0f,0.0f};  // Velocidad hacia delante
        msg.yaw = 1.0f;
        trajectory_setpoint_pub_->publish(msg);
    }

    void arm()
    {
        send_vehicle_command(400, 1.0);  // VEHICLE_CMD_COMPONENT_ARM_DISARM
        RCLCPP_INFO(this->get_logger(), "Enviando comando ARM");
    }

    void takeoff(float altitude)
    {
    send_takeoff_command(22, 0.0f, 0.0f, 0.0f, NAN, NAN, NAN, altitude);
    RCLCPP_INFO(this->get_logger(), "Enviando comando TAKEOFF a %.2f metros", altitude);
    }

    void set_offboard_mode()
    {
        send_vehicle_command(176, 6.0);  // VEHICLE_CMD_DO_SET_MODE, modo 6 = OFFBOARD
        RCLCPP_INFO(this->get_logger(), "Enviando comando OFFBOARD");
    }

    void send_vehicle_command(uint16_t command, float param1)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = timestamp_;
        msg.param1 = param1;
        msg.param2 = 0.0;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        vehicle_command_pub_->publish(msg);
    }

    void send_takeoff_command(
    uint16_t command,
    float param1,
    float param2 = 0.0f,
    float param3 = 0.0f,
    float param4 = 0.0f,
    float param5 = 0.0f,
    float param6 = 0.0f,
    float param7 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp = timestamp_;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        msg.command = command;
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

    uint64_t timestamp_;
    int count_;
    bool init = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
