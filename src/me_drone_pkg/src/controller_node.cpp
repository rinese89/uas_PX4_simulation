#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cmath>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

class DroneMissionClient : public rclcpp::Node
{
public:
    DroneMissionClient() : Node("drone_mission_client")
    {
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<double>("radius", 1.0);
        this->declare_parameter<double>("omega", 2.0);
        this->declare_parameter<double>("height", 5.0);
        this->declare_parameter<double>("center_x", 0.0);
        this->declare_parameter<double>("center_y", 0.0);

        // PID
        this->declare_parameter<double>("kp_x", 0.8);
        this->declare_parameter<double>("ki_x", 0.0);
        this->declare_parameter<double>("kd_x", 0.1);

        this->declare_parameter<double>("kp_y", 0.8);
        this->declare_parameter<double>("ki_y", 0.0);
        this->declare_parameter<double>("kd_y", 0.1);

        this->declare_parameter<double>("kp_z", 0.8);
        this->declare_parameter<double>("ki_z", 0.0);
        this->declare_parameter<double>("kd_z", 0.1);

        this->declare_parameter<double>("max_vx", 100.0);
        this->declare_parameter<double>("max_vy", 100.0);
        this->declare_parameter<double>("max_vz", 100.0);

        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("radius", radius_);
        this->get_parameter("omega", omega_);
        this->get_parameter("height", height_);
        this->get_parameter("center_x", center_x_);
        this->get_parameter("center_y", center_y_);

        this->get_parameter("kp_x", kp_x_);
        this->get_parameter("ki_x", ki_x_);
        this->get_parameter("kd_x", kd_x_);

        this->get_parameter("kp_y", kp_y_);
        this->get_parameter("ki_y", ki_y_);
        this->get_parameter("kd_y", kd_y_);

        this->get_parameter("kp_z", kp_z_);
        this->get_parameter("ki_z", ki_z_);
        this->get_parameter("kd_z", kd_z_);

        this->get_parameter("max_vx", max_vx_);
        this->get_parameter("max_vy", max_vy_);
        this->get_parameter("max_vz", max_vz_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        control_client_ = this->create_client<std_srvs::srv::Trigger>(
            ns_prefix_ + "/enable_control");

        ocm_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            ns_prefix_ + "/fmu/in/offboard_control_mode", 10);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            ns_prefix_ + "/fmu/in/trajectory_setpoint", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ns_prefix_ + "/odom",
            10,
            std::bind(&DroneMissionClient::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&DroneMissionClient::tick, this));

        RCLCPP_INFO(this->get_logger(), "Mission client iniciado en ns=%s", drone_ns_.c_str());
    }

private:
    uint64_t now_us()
    {
        return this->now().nanoseconds() / 1000;
    }

    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    void request_control()
    {
        if (control_requested_) {
            return;
        }

        if (!control_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "[%s] Servicio enable_control no disponible",
                        drone_ns_.c_str());
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

        control_client_->async_send_request(
            req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                try {
                    auto res = future.get();
                    if (res->success) {
                        control_granted_ = true;
                        RCLCPP_INFO(this->get_logger(), "[%s] Control concedido", drone_ns_.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(), "[%s] Control denegado: %s",
                                    drone_ns_.c_str(), res->message.c_str());
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(), "[%s] Error llamando al servicio: %s",
                                 drone_ns_.c_str(), e.what());
                }
            });

        control_requested_ = true;
    }

    void publish_offboard_control_mode(uint64_t ts)
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = ts;
        msg.position = false;
        msg.velocity = true;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        ocm_pub_->publish(msg);
    }

    void publish_velocity_setpoint(uint64_t ts, float vx, float vy, float vz)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;
        sp.position = {NAN, NAN, NAN};
        sp.velocity = {vx, vy, vz};
        sp.acceleration = {NAN, NAN, NAN};
        sp.yaw = NAN;
        sp.yawspeed = 0.0f;

        setpoint_pub_->publish(sp);
    }

    void compute_and_publish_control(uint64_t ts)
    {
        const double t = this->now().seconds() - t0_;

        // Trayectoria deseada
        const double x_d = center_x_ + radius_ * std::cos(omega_ * t);
        const double y_d = center_y_ + radius_ * std::sin(omega_ * t);
        const double z_d = height_;  // en /odom tu z es positiva hacia arriba

        // Feedforward de velocidad para la circunferencia
        const double vx_ff = -radius_ * omega_ * std::sin(omega_ * t);
        const double vy_ff =  radius_ * omega_ * std::cos(omega_ * t);
        const double vz_ff = 0.0;

        // Error
        const double ex = x_d - current_x_;
        const double ey = y_d - current_y_;
        const double ez = z_d - current_z_;

        // dt
        const double now_s = this->now().seconds();
        double dt = now_s - last_control_time_;
        if (!pid_initialized_) {
            dt = 0.05;
            pid_initialized_ = true;
        }
        last_control_time_ = now_s;

        if (dt <= 1e-6) {
            dt = 0.05;
        }

        // Integral
        ix_ += ex * dt;
        iy_ += ey * dt;
        iz_ += ez * dt;

        // Derivada
        const double dex = (ex - prev_ex_) / dt;
        const double dey = (ey - prev_ey_) / dt;
        const double dez = (ez - prev_ez_) / dt;

        prev_ex_ = ex;
        prev_ey_ = ey;
        prev_ez_ = ez;

        // PID + feedforward
        double vx = vx_ff + kp_x_ * ex + ki_x_ * ix_ + kd_x_ * dex;
        double vy = vy_ff + kp_y_ * ey + ki_y_ * iy_ + kd_y_ * dey;
        double vz = vz_ff + kp_z_ * ez + ki_z_ * iz_ + kd_z_ * dez;

        // Saturación
        vx = clamp(vx, -max_vx_, max_vx_);
        vy = clamp(vy, -max_vy_, max_vy_);
        vz = clamp(vz, -max_vz_, max_vz_);

        // Publicar
        publish_offboard_control_mode(ts);

        // PX4 NED: z velocidad positiva baja, negativa sube.
        // Tu /odom tiene z positiva hacia arriba.
        publish_velocity_setpoint(
            ts,
            static_cast<float>(vx),
            static_cast<float>(vy),
            static_cast<float>(-vz)
        );
    }

    void tick()
    {
        request_control();

        if (!control_granted_ || !has_odom_) {
            return;
        }

        uint64_t ts = now_us();

        if (!trajectory_started_) {
            t0_ = this->now().seconds();
            last_control_time_ = t0_;
            trajectory_started_ = true;
            RCLCPP_INFO(this->get_logger(), "[%s] Trayectoria iniciada", drone_ns_.c_str());
        }

        compute_and_publish_control(ts);
    }

    std::string drone_ns_;
    std::string ns_prefix_;

    double radius_{1.0};
    double omega_{2.0};
    double height_{5.0};
    double center_x_{0.0};
    double center_y_{0.0};

    double kp_x_{0.8}, ki_x_{0.0}, kd_x_{0.1};
    double kp_y_{0.8}, ki_y_{0.0}, kd_y_{0.1};
    double kp_z_{0.8}, ki_z_{0.0}, kd_z_{0.1};

    double max_vx_{100.0};
    double max_vy_{100.0};
    double max_vz_{100.0};

    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};
    bool has_odom_{false};

    bool control_requested_{false};
    bool control_granted_{false};
    bool trajectory_started_{false};

    double t0_{0.0};
    double last_control_time_{0.0};
    bool pid_initialized_{false};

    double ix_{0.0}, iy_{0.0}, iz_{0.0};
    double prev_ex_{0.0}, prev_ey_{0.0}, prev_ez_{0.0};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr control_client_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneMissionClient>());
    rclcpp::shutdown();
    return 0;
}