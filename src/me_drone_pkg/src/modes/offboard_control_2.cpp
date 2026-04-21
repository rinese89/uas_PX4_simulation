#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <limits>
using namespace std::chrono_literals;

class AttitudeOffboard : public rclcpp::Node {
public:
  AttitudeOffboard() : Node("att_offboard") {
    ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    att_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint_v1", 10);
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    timer_ = create_wall_timer(50ms, std::bind(&AttitudeOffboard::tick, this)); // 20 Hz
  }

private:
  uint64_t now_us() const { return this->get_clock()->now().nanoseconds()/1000; }

  void tick() {
    const uint64_t ts = now_us();

    // 1) OffboardControlMode: actitud
    px4_msgs::msg::OffboardControlMode ocm{};
    ocm.timestamp = ts;
    ocm.position = false; ocm.velocity = false; ocm.acceleration = false;
    ocm.attitude = true;  ocm.body_rate = false;
    // Si tu px4_msgs tiene estos campos, déjalos en false:
    // ocm.thrust_and_torque = false; ocm.direct_actuator = false;
    ocm_pub_->publish(ocm);

    // 2) Setpoint de actitud + thrust
    //   RPY de referencia (rad): ajusta aquí tu controlador
    const float roll  =  0.0f;
    const float pitch = -5.0f * static_cast<float>(M_PI)/180.0f; // inclina hacia delante
    const float yaw   =  0.0f;                                   // rumbo deseado
    const float throttle = 0.5f; // [0..1] aprox hover en SITL; AJUSTA según tu X500 real

    tf2::Quaternion q; q.setRPY(roll, pitch, yaw); // orden RPY intrínseco (X-Y-Z)
    px4_msgs::msg::VehicleAttitudeSetpoint sp{};
    sp.timestamp = ts;
    sp.q_d = {static_cast<float>(q.w()), static_cast<float>(q.x()),
              static_cast<float>(q.y()), static_cast<float>(q.z())};
    sp.yaw_sp_move_rate = 0.0f; // opcional (feedforward en rad/s, en Tierra)
    // thrust_body en FRD normalizado [-1,1]: Z negativo "hacia arriba"
    sp.thrust_body = {0.0f, 0.0f, -std::clamp(throttle, 0.0f, 1.0f)};
    att_pub_->publish(sp);

    // 3) Secuencia de modo y armado tras ~1 s de streaming
    if (++count_ == 20) set_offboard();  // VEHICLE_CMD_DO_SET_MODE (OFFBOARD)
    if (count_ == 25) arm();             // ARM
  }

  void set_offboard() {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = now_us();
    cmd.command = 176;    // VEHICLE_CMD_DO_SET_MODE
    cmd.param1 = 1.0f;    // custom mode
    cmd.param2 = 6.0f;    // OFFBOARD
    cmd.param3 = 0.0f;
    cmd.target_system = 1; cmd.target_component = 1;
    cmd.source_system = 1; cmd.source_component = 1;
    cmd.from_external = true;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "OFFBOARD solicitado");
  }

  void arm() {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = now_us();
    cmd.command = 400;  // VEHICLE_CMD_COMPONENT_ARM_DISARM
    cmd.param1 = 1.0f;  // arm
    cmd.target_system = 1; cmd.target_component = 1;
    cmd.source_system = 1; cmd.source_component = 1;
    cmd.from_external = true;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Armando...");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr att_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
  int count_ = 0;
};
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AttitudeOffboard>());
  rclcpp::shutdown();
  return 0;
}
