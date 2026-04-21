#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <termios.h>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

class AttitudeTeleop : public rclcpp::Node
{
public:
    AttitudeTeleop() : Node("attitude_teleop")
    {
        attitude_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", 10);
        ocm_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        timer_ = create_wall_timer(50ms, std::bind(&AttitudeTeleop::tick, this));

        // Terminal sin bloqueo
        tcgetattr(STDIN_FILENO, &oldt_);
        newt_ = oldt_;
        newt_.c_lflag &= ~(ICANON | ECHO);
        newt_.c_cc[VMIN]  = 0;
        newt_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &newt_);

        print_help();
    }

    ~AttitudeTeleop()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
    }

private:
    void print_help()
    {
        RCLCPP_INFO(get_logger(), "--- Attitude Teleop ---");
        RCLCPP_INFO(get_logger(), "W/S  : pitch adelante/atrás");
        RCLCPP_INFO(get_logger(), "A/D  : roll izquierda/derecha");
        RCLCPP_INFO(get_logger(), "Q/E  : yaw izquierda/derecha");
        RCLCPP_INFO(get_logger(), "U/J  : throttle subir/bajar");
        RCLCPP_INFO(get_logger(), "SPACE: nivel (roll=0, pitch=0)");
        RCLCPP_INFO(get_logger(), "X    : arm  |  Z: disarm");
    }

    uint64_t now_us()
    {
        return get_clock()->now().nanoseconds() / 1000;
    }

    void send_command(uint16_t cmd, float p1, float p2 = 0)
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
        cmd_pub_->publish(msg);
    }

    // Convierte roll/pitch/yaw (radianes, ENU) a cuaternión NED para PX4
    std::array<float, 4> rpy_to_quaternion_ned(float roll, float pitch, float yaw)
    {
        // PX4 espera cuaternión en NED: convertimos yaw invirtiendo signo
        float cy = cosf(-yaw * 0.5f);
        float sy = sinf(-yaw * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);

        return {
            cr * cp * cy + sr * sp * sy,   // w
            sr * cp * cy - cr * sp * sy,   // x
            cr * sp * cy + sr * cp * sy,   // y
            cr * cp * sy - sr * sp * cy    // z
        };
    }

    void tick()
    {
        uint64_t ts = now_us();
        count_++;

        // Leer tecla
        char c = 0;
        read(STDIN_FILENO, &c, 1);

        const float roll_step     = 0.05f;   // ~3 grados
        const float pitch_step    = 0.05f;
        const float yaw_step      = 0.05f;
        const float throttle_step = 0.02f;

        switch (c) {
            case 'w': pitch_ -= pitch_step; break;   // nariz abajo = adelante
            case 's': pitch_ += pitch_step; break;
            case 'a': roll_  -= roll_step;  break;
            case 'd': roll_  += roll_step;  break;
            case 'q': yaw_   -= yaw_step;   break;
            case 'e': yaw_   += yaw_step;   break;
            case 'u': throttle_ = fminf(throttle_ + throttle_step, 1.0f); break;
            case 'j': throttle_ = fmaxf(throttle_ - throttle_step, 0.0f); break;
            case ' ':
                roll_  = 0.0f;
                pitch_ = 0.0f;
                break;
            case 'x':
                send_command(176, 1.0f, 6.0f);  // OFFBOARD
                rclcpp::sleep_for(200ms);
                send_command(400, 1.0f);         // ARM
                RCLCPP_INFO(get_logger(), "ARM + OFFBOARD");
                break;
            case 'z':
                send_command(400, 0.0f);         // DISARM
                RCLCPP_INFO(get_logger(), "DISARM");
                break;
            default: break;
        }

        // Limitar ángulos a ±30 grados
        const float max_angle = 0.52f;  // ~30 deg en radianes
        roll_  = fmaxf(-max_angle, fminf(max_angle, roll_));
        pitch_ = fmaxf(-max_angle, fminf(max_angle, pitch_));

        // Offboard control mode: attitude=true
        px4_msgs::msg::OffboardControlMode ocm{};
        ocm.timestamp    = ts;
        ocm.attitude     = true;
        ocm.body_rate    = false;
        ocm.velocity     = false;
        ocm.position     = false;
        ocm.acceleration = false;
        ocm_pub_->publish(ocm);

        // Attitude setpoint
        px4_msgs::msg::VehicleAttitudeSetpoint att{};
        att.timestamp = ts;

        auto q = rpy_to_quaternion_ned(roll_, pitch_, yaw_);
        att.q_d[0] = q[0];  // w
        att.q_d[1] = q[1];  // x
        att.q_d[2] = q[2];  // y
        att.q_d[3] = q[3];  // z

        att.thrust_body[0] = 0.0f;
        att.thrust_body[1] = 0.0f;
        att.thrust_body[2] = -throttle_;  // NED: empuje negativo = hacia arriba

        attitude_pub_->publish(att);

        // Log periódico
        if (count_ % 20 == 0) {
            RCLCPP_INFO(get_logger(),
                "roll: %.2f°  pitch: %.2f°  yaw: %.2f°  throttle: %.2f",
                roll_  * 180.0f / M_PI,
                pitch_ * 180.0f / M_PI,
                yaw_   * 180.0f / M_PI,
                throttle_);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;

    float roll_{0.0f}, pitch_{0.0f}, yaw_{0.0f};
    float throttle_{0.5f};  // arrancar con 50% de empuje
    int count_{0};

    struct termios oldt_, newt_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudeTeleop>());
    rclcpp::shutdown();
    return 0;
}