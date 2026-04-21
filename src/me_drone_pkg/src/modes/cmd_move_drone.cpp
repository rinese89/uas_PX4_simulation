#include "rclcpp/rclcpp.hpp"
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
        // Parámetros (opcionales desde launch/CLI)
        target_alt_m_   = declare_parameter<double>("target_alt_m", 1.0);    // altura deseada [m]
        vz_takeoff_mps_ = declare_parameter<double>("vz_takeoff_mps", 0.5);  // módulo de velocidad de ascenso [m/s]
        alt_tol_m_      = declare_parameter<double>("alt_tol_m", 0.05);      // tolerancia [m]

        trajectory_setpoint_pub_ =
            create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);

        // QoS recomendado para datos provenientes del agente (suele ser best-effort)
        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.durability_volatile();

        odom_drone_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry",
            qos,
            std::bind(&CmdMoveDrone::odomCb, this, std::placeholders::_1)
        );

        // 20 Hz
        timer_ = create_wall_timer(50ms, std::bind(&CmdMoveDrone::tick, this));

        timestamp_ = now_us();
        RCLCPP_INFO(get_logger(), "Offboard node listo. Objetivo: %.2f m (vz=%.2f m/s, tol=%.2f m)",
                    target_alt_m_, vz_takeoff_mps_, alt_tol_m_);
    }

private:
    // ===== Utilidades =====
    uint64_t now_us() { return this->get_clock()->now().nanoseconds() / 1000; }

    // ===== Callback de odometría =====
    void odomCb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        last_pose_frame_ = msg->pose_frame; // por si quieres verificar el marco
        // En NED: z positiva es abajo. Altura (AGL) ~ -z (si origen ≈ punto de despegue)
        current_alt_m_ = -static_cast<double>(msg->position[2]);
        have_odom_ = true;
    }

    // ===== Lazo de control simple =====
    void tick()
    {
        timestamp_ = now_us();

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = timestamp_;
        // Control por VELOCIDAD: deja position/accel en NaN
        sp.position = {NAN, NAN, NAN};
        sp.acceleration = {NAN, NAN, NAN};
        sp.yaw = std::numeric_limits<float>::quiet_NaN();

        if (!have_odom_) {
            // Hasta recibir odometría, no intentamos subir fuerte. Opcionalmente, puedes mantener 0.
            sp.velocity = {0.0f, 0.0f, 0.0f};
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Esperando odometría...");
        } else {
            // Estado basado en altura
            const double err = target_alt_m_ - current_alt_m_;

            if (!reached_alt_) {
                // Aún no hemos llegado: subir (NED: z < 0 es subir)
                sp.velocity = {0.0f, 0.0f, static_cast<float>(-std::fabs(vz_takeoff_mps_))};

                // Check de llegada con tolerancia e histéresis simple
                if (err <= alt_tol_m_) {
                    reached_alt_ = true;
                    RCLCPP_INFO(get_logger(), "Altura alcanzada: %.2f m (objetivo %.2f ± %.2f)",
                                current_alt_m_, target_alt_m_, alt_tol_m_);
                } else {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "Takeoff: alt=%.2f m -> objetivo=%.2f m (faltan %.2f m)",
                                         current_alt_m_, target_alt_m_, err);
                }
            } else {
                // Mantener (velocidad cero). Si quieres, aquí puedes añadir un control fino de z.
                sp.velocity = {0.0f, 0.0f, 0.0f};
                // Histéresis: si cae por debajo de (target - 2*tol), volver a subir
                if (current_alt_m_ < (target_alt_m_ - 2.0 * alt_tol_m_)) {
                    reached_alt_ = false;
                    RCLCPP_WARN(get_logger(),
                                "Altura por debajo del umbral de histéresis (%.2f m). Reintentando ascenso.",
                                current_alt_m_);
                }
            }
        }

        trajectory_setpoint_pub_->publish(sp);
    }

    // ===== Miembros =====
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_drone_sub_;

    uint64_t timestamp_{};
    bool have_odom_{false};
    bool reached_alt_{false};
    int last_pose_frame_{0};  // Por si quieres comprobar que es NED
    double current_alt_m_{0.0};

    // Parámetros
    double target_alt_m_{1.0};
    double vz_takeoff_mps_{0.5};
    double alt_tol_m_{0.05};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdMoveDrone>());
    rclcpp::shutdown();
    return 0;
}
