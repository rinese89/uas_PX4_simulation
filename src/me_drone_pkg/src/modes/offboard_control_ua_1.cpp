#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl() : Node("offboard_control_node")
    {
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        state_pub_ = create_publisher<std_msgs::msg::String>(
            "/drone_state", 10);

        rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.best_effort();
        odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&OffboardControl::odom_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(50ms, std::bind(&OffboardControl::tick, this));

        RCLCPP_INFO(get_logger(), "Offboard control iniciado. Despegando a %.1f m...", target_alt_);
    }

private:
    uint64_t now_us()
    {
        return get_clock()->now().nanoseconds() / 1000;
    }

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Altitud actual en metros (NED: z negativo = arriba, invertimos)
        current_alt_ = -msg->position[2];
    }

    void publish_state(const std::string & s)
    {
        std_msgs::msg::String msg;
        msg.data = s;
        state_pub_->publish(msg);
    }

    void send_vehicle_command(uint16_t cmd, float p1, float p2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand msg{};
        msg.timestamp        = now_us();
        msg.command          = cmd;
        msg.param1           = p1;
        msg.param2           = p2;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        vehicle_command_pub_->publish(msg);
    }

    void tick()
    {
        uint64_t ts = now_us();
        count_++;

        // Siempre publicar los tres topics mientras este nodo es dueño
        if (state_ == "takeoff") {
            publish_offboard_control_mode(ts);
            publish_takeoff_setpoint(ts);
            publish_state("takeoff");
        }

        // Secuencia de arranque
        if (count_ == 50)  send_vehicle_command(176, 1.0f, 6.0f);  // OFFBOARD
        if (count_ == 100) send_vehicle_command(400, 1.0f);         // ARM

        // Comprobar si hemos alcanzado la altura objetivo
        if (state_ == "takeoff" && count_ > 100) {
            float error = std::abs(current_alt_ - target_alt_);
            if (error < 0.3f) {
                RCLCPP_INFO(get_logger(),
                    "Altura alcanzada: %.2f m. Pasando control a teleop.", current_alt_);
                state_ = "teleop";
                publish_state("teleop");
            }
        }

        // Log periódico
        if (count_ % 20 == 0) {
            RCLCPP_INFO(get_logger(), "Alt: %.2f m | Estado: %s", current_alt_, state_.c_str());
        }
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
        ocm_pub_->publish(msg);
    }

    void publish_takeoff_setpoint(uint64_t ts)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp    = ts;
        sp.velocity     = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        sp.yaw          = 0.0f;
        sp.position     = {0.0f, 0.0f, -target_alt_};  // NED
        setpoint_pub_->publish(sp);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;

    std::string state_{"takeoff"};
    float current_alt_{0.0f};
    float target_alt_{5.0f};  // metros
    int count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}