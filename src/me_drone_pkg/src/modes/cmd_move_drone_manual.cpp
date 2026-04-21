#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <limits>
#include <cmath>

using namespace std::chrono_literals;

class CmdMoveDrone : public rclcpp::Node
{
public:
    CmdMoveDrone() : Node("offboard_control_node")
    {
        trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.durability_volatile();

        odom_drone_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&CmdMoveDrone::odomCb, this, std::placeholders::_1)
        );

        // 20 Hz recomendado
        timer_ = create_wall_timer(50ms, std::bind(&CmdMoveDrone::tick, this));

        timestamp_ = now_us();
    }

private:
    uint64_t now_us() const { return this->get_clock()->now().nanoseconds() / 1000; }

    void odomCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        last_pose_frame_ = msg->pose_frame; // por si quieres verificar el marco
        // En NED: z positiva es abajo. Altura (AGL) ~ -z (si origen ≈ punto de despegue)
        current_alt_m_ = -static_cast<double>(msg->position[2]);
        have_odom_ = true;
    }

    void tick()
    {
        timestamp_ = now_us();
        count_++;
        
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp =timestamp_;     // o now_us() si ya te funciona estable
        sp.position = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        if (altitude_ < 1.0) {
          // Despegue OFFBOARD durante ~5 s a -0.5 m/s
          sp.velocity = {0.0f, 0.0f, -0.5f}; // NED: z<0 es subir
          sp.yaw = NAN;                      // no controlar yaw
          RCLCPP_INFO(this->get_logger(),"Taking off");
        } 

        else {
          // Stalibization
          sp.velocity = {0.0f, 0.0f, 0.0f}; // NED: z<0 es subir
          sp.yaw = NAN;                      // no controlar yaw
          RCLCPP_INFO(this->get_logger(),"Stabilizing drone in the air");

        } 
        
        //else if (count_ >= 350 && count_ <450) {
        //  // Avance continuo
        //  sp.velocity = {1.0f, 0.0f, 0.0f};  // 1 m/s hacia el Norte local
        //  sp.yaw = NAN;                      // no imponer rumbo
        //  RCLCPP_INFO(this->get_logger(),"Moving forward");
        //}

      //  else {
      //    // Stop
      //    sp.velocity = {0.0f, 0.0f, 0.0f};  // 1 m/s hacia el Norte local
      //    sp.yaw = NAN;                      // no imponer rumbo
      //    RCLCPP_INFO(this->get_logger(),"Moving forward");
      //  }


        trajectory_setpoint_pub_->publish(sp);
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


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_drone_sub_;
    uint64_t timestamp_{};
    int count_{0};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdMoveDrone>());
    rclcpp::shutdown();
    return 0;
}
