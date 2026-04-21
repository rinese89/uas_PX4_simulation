#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cmath>
#include <string>

using namespace std::chrono_literals;

class DroneMissionClient : public rclcpp::Node
{
public:
    DroneMissionClient() : Node("drone_mission_client")
    {
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<double>("radius", 1.0);
        this->declare_parameter<double>("omega", 0.2);
        this->declare_parameter<double>("height", 5.0);
        this->declare_parameter<double>("center_x", 0.0);
        this->declare_parameter<double>("center_y", 0.0);

        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("radius", radius_);
        this->get_parameter("omega", omega_);
        this->get_parameter("height", height_);
        this->get_parameter("center_x", center_x_);
        this->get_parameter("center_y", center_y_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        control_client_ = this->create_client<std_srvs::srv::Trigger>(
            ns_prefix_ + "/enable_control");

        ocm_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            ns_prefix_ + "/fmu/in/offboard_control_mode", 10);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            ns_prefix_ + "/fmu/in/trajectory_setpoint", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&DroneMissionClient::tick, this));

        RCLCPP_INFO(this->get_logger(), "Mission client iniciado en ns=%s", drone_ns_.c_str());
    }

private:
    uint64_t now_us()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
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
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        ocm_pub_->publish(msg);
    }

    void publish_circular_setpoint(uint64_t ts)
    {
        const double t = (this->now().seconds() - t0_);
        const double x = center_x_ + radius_ * std::cos(omega_ * t);
        const double y = center_y_ + radius_ * std::sin(omega_ * t);
        const double z = -height_;  // PX4 NED

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;
        sp.position = {
            static_cast<float>(x),
            static_cast<float>(y),
            static_cast<float>(z)
        };
        sp.velocity = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        sp.yaw = 0.0f;

        setpoint_pub_->publish(sp);
    }

    void tick()
    {
        request_control();

        if (!control_granted_) {
            return;
        }

        uint64_t ts = now_us();

        if (!trajectory_started_) {
            t0_ = this->now().seconds();
            trajectory_started_ = true;
        }

        publish_offboard_control_mode(ts);
        publish_circular_setpoint(ts);
    }

    std::string drone_ns_;
    std::string ns_prefix_;

    double radius_{1.0};
    double omega_{0.2};
    double height_{5.0};
    double center_x_{0.0};
    double center_y_{0.0};

    bool control_requested_{false};
    bool control_granted_{false};
    bool trajectory_started_{false};
    double t0_{0.0};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr control_client_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneMissionClient>());
    rclcpp::shutdown();
    return 0;
}