#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <termios.h>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

class KeyboardTeleop : public rclcpp::Node
{
public:
    KeyboardTeleop() : Node("keyboard_teleop")
    {
        // =========================
        // PARÁMETRO NAMESPACE
        // =========================
        this->declare_parameter<std::string>("drone_ns", "");
        this->get_parameter("drone_ns", drone_ns_);

        auto ns_prefix = drone_ns_.empty() ? "" : "/" + drone_ns_;

        RCLCPP_INFO(get_logger(), "Teleop namespace: %s", ns_prefix.c_str());

        // =========================
        // PUBS
        // =========================
        setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            ns_prefix + "/fmu/in/trajectory_setpoint", 10);

        ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            ns_prefix + "/fmu/in/offboard_control_mode", 10);

        // =========================
        // SUBS
        // =========================
        state_sub_ = create_subscription<std_msgs::msg::String>(
            ns_prefix + "/drone_state", 10,
            std::bind(&KeyboardTeleop::state_callback, this, std::placeholders::_1));

        // =========================
        // CLIENTE DE SERVICIO
        // =========================
        teleop_client_ = create_client<std_srvs::srv::Trigger>(
            ns_prefix + "/enable_control");

        // =========================
        // TIMER
        // =========================
        timer_ = create_wall_timer(50ms, std::bind(&KeyboardTeleop::tick, this));

        // =========================
        // TERMINAL RAW
        // =========================
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        newt_.c_cc[VMIN]  = 0;
        newt_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);

        RCLCPP_INFO(get_logger(), "Pulsa 't' para tomar control del dron");
    }

    ~KeyboardTeleop()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

private:
    std::string drone_ns_;

    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "control" && drone_state_ != "control") {
            RCLCPP_INFO(get_logger(), "Control recibido. Teleop activo.");
        }
        drone_state_ = msg->data;
    }

    uint64_t now_us()
    {
        return get_clock()->now().nanoseconds() / 1000;
    }

    // =========================
    // LLAMADA A SERVICIO
    // =========================
    void request_teleop()
    {
        if (!teleop_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Servicio no disponible");
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

        teleop_client_->async_send_request(req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                auto res = future.get();
                if (res->success) {
                    RCLCPP_INFO(get_logger(), "Teleop concedido");
                } else {
                    RCLCPP_WARN(get_logger(), "Teleop rechazado");
                }
            });
    }

    void tick()
    {
        uint64_t ts = now_us();

        char c = 0;
        read(STDIN_FILENO, &c, 1);

        // =========================
        // PEDIR CONTROL
        // =========================
        if (c == 't') {
            RCLCPP_INFO(get_logger(), "Solicitando control...");
            request_teleop();
            return;
        }

        // =========================
        // SOLO CONTROL SI TENEMOS PERMISO
        // =========================
        if (drone_state_ != "control") return;

        const float speed = 2.0f;
        const float yaw_speed = 1.0f;

        switch (c) {
            case 'w': vx_ =  speed; vy_ = 0.0f;  break;
            case 's': vx_ = -speed; vy_ = 0.0f;  break;
            case 'a': vy_ = -speed; vx_ = 0.0f;  break;
            case 'd': vy_ =  speed; vx_ = 0.0f;  break;
            case 'r': vz_ = -speed; break;
            case 'f': vz_ =  speed; break;
            case 'q': yaw_rate_ = yaw_speed; break;
            case 'e': yaw_rate_ = -yaw_speed; break;

            case ' ':
                vx_ = vy_ = vz_ = 0.0f;
                yaw_rate_ = 0.0f;
                RCLCPP_INFO(get_logger(), "HOVER");
                break;

            default:
                vx_ *= 0.8f;
                vy_ *= 0.8f;
                vz_ *= 0.8f;
                yaw_rate_ *= 0.8f;
                break;
        }

        // =========================
        // OFFBOARD
        // =========================
        px4_msgs::msg::OffboardControlMode ocm{};
        ocm.timestamp = ts;
        ocm.velocity  = true;
        ocm_pub_->publish(ocm);

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;
        sp.position  = {NAN, NAN, NAN};
        sp.velocity  = {vx_, vy_, vz_};
        sp.yawspeed  = yaw_rate_;

        setpoint_pub_->publish(sp);
    }

    // =========================
    // MEMBERS
    // =========================
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr teleop_client_;

    std::string drone_state_{"takeoff"};

    float vx_{0.0f}, vy_{0.0f}, vz_{0.0f};
    float yaw_rate_{0.0f};

    struct termios oldt_, newt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyboardTeleop>());
    rclcpp::shutdown();
    return 0;
}