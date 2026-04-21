#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
#include <functional>

using namespace std::chrono_literals;

class DroneWaypointMissionClient : public rclcpp::Node
{
public:
    DroneWaypointMissionClient() : Node("drone_waypoint_mission_client")
    {
        // =========================
        // PARÁMETROS
        // =========================
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<std::string>("odom_topic", "odom");
        this->declare_parameter<std::string>("marker_frame", "map");

        // Altura fija de vuelo
        this->declare_parameter<double>("height", 5.0);

        // Tolerancias
        this->declare_parameter<double>("goal_tolerance", 0.30);
        this->declare_parameter<double>("slowdown_radius", 1.5);

        // Waypoints XY
        this->declare_parameter<double>("p1_x", 0.0);
        this->declare_parameter<double>("p1_y", 0.0);

        this->declare_parameter<double>("p2_x", 3.0);
        this->declare_parameter<double>("p2_y", 0.0);

        this->declare_parameter<double>("p3_x", 3.0);
        this->declare_parameter<double>("p3_y", 3.0);

        this->declare_parameter<double>("p4_x", 0.0);
        this->declare_parameter<double>("p4_y", 3.0);

        // PID XY
        this->declare_parameter<double>("kp_x", 0.9);
        this->declare_parameter<double>("ki_x", 0.0);
        this->declare_parameter<double>("kd_x", 0.15);

        this->declare_parameter<double>("kp_y", 0.9);
        this->declare_parameter<double>("ki_y", 0.0);
        this->declare_parameter<double>("kd_y", 0.15);

        // PID Z
        this->declare_parameter<double>("kp_z", 0.8);
        this->declare_parameter<double>("ki_z", 0.0);
        this->declare_parameter<double>("kd_z", 0.10);

        // Saturaciones
        this->declare_parameter<double>("max_vx", 1.5);
        this->declare_parameter<double>("max_vy", 1.5);
        this->declare_parameter<double>("max_vz", 0.8);
        this->declare_parameter<double>("max_speed_xy", 1.8);

        // Anti-windup
        this->declare_parameter<double>("integral_limit_x", 2.0);
        this->declare_parameter<double>("integral_limit_y", 2.0);
        this->declare_parameter<double>("integral_limit_z", 2.0);

        // Frecuencia control
        this->declare_parameter<double>("control_period_ms", 50.0);

        // =========================
        // LECTURA DE PARÁMETROS
        // =========================
        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("marker_frame", marker_frame_);

        this->get_parameter("height", height_);
        this->get_parameter("goal_tolerance", goal_tolerance_);
        this->get_parameter("slowdown_radius", slowdown_radius_);

        this->get_parameter("p1_x", p1_[0]);
        this->get_parameter("p1_y", p1_[1]);

        this->get_parameter("p2_x", p2_[0]);
        this->get_parameter("p2_y", p2_[1]);

        this->get_parameter("p3_x", p3_[0]);
        this->get_parameter("p3_y", p3_[1]);

        this->get_parameter("p4_x", p4_[0]);
        this->get_parameter("p4_y", p4_[1]);

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
        this->get_parameter("max_speed_xy", max_speed_xy_);

        this->get_parameter("integral_limit_x", integral_limit_x_);
        this->get_parameter("integral_limit_y", integral_limit_y_);
        this->get_parameter("integral_limit_z", integral_limit_z_);

        this->get_parameter("control_period_ms", control_period_ms_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        // =========================
        // WAYPOINTS
        // =========================
        forward_points_.push_back(p1_);
        forward_points_.push_back(p2_);
        forward_points_.push_back(p3_);
        forward_points_.push_back(p4_);

        // =========================
        // COMUNICACIONES
        // =========================
        control_client_ = this->create_client<std_srvs::srv::Trigger>(
            ns_prefix_ + "/enable_control");

        ocm_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            ns_prefix_ + "/fmu/in/offboard_control_mode", 10);

        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            ns_prefix_ + "/fmu/in/trajectory_setpoint", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            ns_prefix_ + "/mission_markers", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            ns_prefix_ + "/" + odom_topic_,
            10,
            std::bind(&DroneWaypointMissionClient::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(control_period_ms_)),
            std::bind(&DroneWaypointMissionClient::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "Nodo iniciado. ns='%s' | odom='%s' | frame markers='%s'",
                    drone_ns_.c_str(), odom_topic_.c_str(), marker_frame_.c_str());
    }

private:
    // ============================================================
    // UTILIDADES
    // ============================================================
    uint64_t now_us() const
    {
        return this->now().nanoseconds() / 1000;
    }

    static double clamp(double value, double low, double high)
    {
        return std::max(low, std::min(value, high));
    }

    static double norm2d(double x, double y)
    {
        return std::sqrt(x * x + y * y);
    }

    static geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    static double nan_f64()
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // ============================================================
    // CALLBACKS
    // ============================================================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        has_odom_ = true;
    }

    // ============================================================
    // CONTROL EXTERNO
    // ============================================================
    void request_control()
    {
        if (control_requested_) {
            return;
        }

        if (!control_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(),
                        "[%s] Servicio enable_control no disponible",
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
                        RCLCPP_INFO(this->get_logger(),
                                    "[%s] Control concedido",
                                    drone_ns_.c_str());
                    } else {
                        RCLCPP_WARN(this->get_logger(),
                                    "[%s] Control denegado: %s",
                                    drone_ns_.c_str(),
                                    res->message.c_str());
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "[%s] Error llamando al servicio: %s",
                                 drone_ns_.c_str(),
                                 e.what());
                }
            });

        control_requested_ = true;
    }

    // ============================================================
    // PUBLICACIÓN PX4
    // ============================================================
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

    // FIX Bug 1 — Conversión correcta ENU (ROS2/odom) → NED (PX4)
    //
    // El odómetro publica en ENU: X=Este, Y=Norte, Z=Arriba.
    // PX4 espera setpoints en NED: X=Norte, Y=Este, Z=Abajo.
    //
    // Mapping:
    //   NED_x =  ENU_y   (Norte ← Y ENU)
    //   NED_y =  ENU_x   (Este  ← X ENU)
    //   NED_z = -ENU_z   (Down  ← -Up)
    //
    // Los errores ex, ey, ez se calculan en ENU (espacio del odómetro),
    // y la conversión se aplica aquí, en el único punto de publicación.
    void publish_velocity_setpoint(uint64_t ts,
                                   double vx_enu, double vy_enu, double vz_enu)
    {
        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.timestamp = ts;

        sp.position = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        // ENU → NED
        sp.velocity = {
            static_cast<float>( vy_enu),   // NED X ←  ENU Y
            static_cast<float>( vx_enu),   // NED Y ←  ENU X
            static_cast<float>(-vz_enu)    // NED Z ← -ENU Z
        };

        sp.acceleration = {
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64()),
            static_cast<float>(nan_f64())
        };

        sp.yaw = static_cast<float>(nan_f64());
        sp.yawspeed = 0.0f;

        setpoint_pub_->publish(sp);
    }

    // ============================================================
    // LÓGICA DE MISIÓN
    // ============================================================
    std::array<double, 2> current_goal() const
    {
        if (!reverse_path_) {
            return forward_points_[goal_index_];
        }
        // Recorrido inverso: p4 → p3 → p2 → p1
        return forward_points_[forward_points_.size() - 1 - goal_index_];
    }

    // FIX Bug 5 — Lógica de avance de waypoints corregida
    //
    // Problema original: cuando goal_index_ alcanzaba el último waypoint,
    // el reset a 0 y el cambio de sentido ocurrían ANTES de contabilizar
    // la visita al último punto, haciendo que se saltara p1 en el sentido
    // inverso.
    //
    // Solución: incrementar siempre goal_index_ y hacer el wrap/flip
    // solo cuando supera el tamaño del vector.
    void advance_goal_if_reached()
    {
        const auto goal = current_goal();

        const double ex = goal[0] - current_x_;
        const double ey = goal[1] - current_y_;
        const double dist = norm2d(ex, ey);

        if (dist > goal_tolerance_) {
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "[%s] Waypoint alcanzado: idx=%zu | objetivo=(%.2f, %.2f)",
                    drone_ns_.c_str(), goal_index_, goal[0], goal[1]);

        // Incrementamos siempre; el wrap ocurre cuando superamos el final
        goal_index_++;

        if (goal_index_ >= forward_points_.size()) {
            goal_index_ = 0;
            reverse_path_ = !reverse_path_;

            RCLCPP_INFO(this->get_logger(),
                        "[%s] Cambio de sentido. reverse_path=%s",
                        drone_ns_.c_str(),
                        reverse_path_ ? "true" : "false");
        }

        reset_pid_state();
    }

    // FIX Bug 3 + Bug 2 — reset_pid_state resetea también last_control_time
    // y activa el flag para suprimir la derivada en el primer tick.
    //
    // Problema original: al cambiar de waypoint, last_control_time_ quedaba
    // con el valor del tick anterior. Si había pasado tiempo (p.ej. el drone
    // tardó en llegar), el primer dt sería enorme, disparando la derivada y
    // la integral en el tick siguiente al reset.
    void reset_pid_state()
    {
        ix_ = 0.0;
        iy_ = 0.0;
        iz_ = 0.0;

        prev_ex_ = 0.0;
        prev_ey_ = 0.0;
        prev_ez_ = 0.0;

        pid_initialized_      = false;
        first_derivative_tick_ = true;                        // FIX Bug 2
        last_control_time_    = this->now().seconds();        // FIX Bug 3
    }

    // ============================================================
    // PID Y PUBLICACIÓN DE CONTROL
    // ============================================================
    void compute_and_publish_control(uint64_t ts)
    {
        const auto goal = current_goal();

        // Errores en ENU (mismo marco que el odómetro)
        const double ex = goal[0] - current_x_;
        const double ey = goal[1] - current_y_;
        const double ez = height_ - current_z_;

        const double dist_xy = norm2d(ex, ey);

        // dt
        const double now_s = this->now().seconds();
        double dt = now_s - last_control_time_;
        if (!pid_initialized_) {
            dt = control_period_ms_ / 1000.0;
            pid_initialized_ = true;
        }
        last_control_time_ = now_s;

        if (dt <= 1e-6) {
            dt = control_period_ms_ / 1000.0;
        }

        // PID integral con anti-windup
        ix_ += ex * dt;
        iy_ += ey * dt;
        iz_ += ez * dt;

        ix_ = clamp(ix_, -integral_limit_x_, integral_limit_x_);
        iy_ = clamp(iy_, -integral_limit_y_, integral_limit_y_);
        iz_ = clamp(iz_, -integral_limit_z_, integral_limit_z_);

        // FIX Bug 2 — Derivada sobre el error, suprimida en el primer tick
        //
        // Problema original: tras reset_pid_state(), prev_ex_=0.
        // En el primer tick dex = (ex - 0) / dt producía un spike
        // proporcional al error actual dividido por dt, lo que generaba
        // saturación inmediata de velocidad en cada cambio de waypoint.
        //
        // Solución: no calcular la derivada en el primer tick tras un reset.
        // A partir del segundo tick, prev_ex_ ya contiene el error real del
        // ciclo anterior y la derivada es correcta.
        double dex = 0.0;
        double dey = 0.0;
        double dez = 0.0;

        if (!first_derivative_tick_) {
            dex = (ex - prev_ex_) / dt;
            dey = (ey - prev_ey_) / dt;
            dez = (ez - prev_ez_) / dt;
        }
        first_derivative_tick_ = false;

        prev_ex_ = ex;
        prev_ey_ = ey;
        prev_ez_ = ez;

        // PID en velocidad (espacio ENU)
        double vx = kp_x_ * ex + ki_x_ * ix_ + kd_x_ * dex;
        double vy = kp_y_ * ey + ki_y_ * iy_ + kd_y_ * dey;
        double vz = kp_z_ * ez + ki_z_ * iz_ + kd_z_ * dez;

        // FIX Bug 4 — Slowdown correcto: limita la velocidad máxima permitida
        //
        // Problema original: se multiplicaba la salida PID completa por alpha,
        // incluyendo el término integral acumulado. Esto no es equivalente a
        // reducir la velocidad; con integral grande el drone podía igualmente
        // llegar rápido o con sobreimpulso.
        //
        // Solución: usar alpha para reducir el límite de velocidad máxima del
        // módulo XY. El PID sigue calculándose completo; solo la saturación
        // final es más restrictiva cerca del waypoint.
        double effective_max_xy = max_speed_xy_;
        if (dist_xy < slowdown_radius_ && slowdown_radius_ > 1e-6) {
            const double alpha = clamp(dist_xy / slowdown_radius_, 0.15, 1.0);
            effective_max_xy *= alpha;
        }

        // Saturación individual por eje
        vx = clamp(vx, -max_vx_, max_vx_);
        vy = clamp(vy, -max_vy_, max_vy_);
        vz = clamp(vz, -max_vz_, max_vz_);

        // Saturación del módulo XY con el límite dinámico de slowdown
        const double vxy = norm2d(vx, vy);
        if (vxy > effective_max_xy && vxy > 1e-9) {
            const double scale = effective_max_xy / vxy;
            vx *= scale;
            vy *= scale;
        }

        publish_offboard_control_mode(ts);

        // La conversión ENU→NED ocurre dentro de publish_velocity_setpoint
        publish_velocity_setpoint(ts, vx, vy, vz);
    }

    // ============================================================
    // MARKERS RVIZ2
    // ============================================================
    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray array;

        // Borrado previo
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = marker_frame_;
            del.header.stamp = this->now();
            del.ns = "mission";
            del.id = 0;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            array.markers.push_back(del);
        }

        // Línea cerrada p1→p2→p3→p4→p1
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = marker_frame_;
            line.header.stamp = this->now();
            line.ns = "mission_path";
            line.id = 1;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.06;
            line.color.r = 0.1f;
            line.color.g = 0.9f;
            line.color.b = 0.2f;
            line.color.a = 1.0f;

            for (const auto & p : forward_points_) {
                line.points.push_back(make_point(p[0], p[1], 0.05));
            }
            line.points.push_back(make_point(forward_points_[0][0], forward_points_[0][1], 0.05));

            array.markers.push_back(line);
        }

        // Puntos de waypoint
        {
            visualization_msgs::msg::Marker pts;
            pts.header.frame_id = marker_frame_;
            pts.header.stamp = this->now();
            pts.ns = "mission_points";
            pts.id = 2;
            pts.type = visualization_msgs::msg::Marker::POINTS;
            pts.action = visualization_msgs::msg::Marker::ADD;
            pts.pose.orientation.w = 1.0;
            pts.scale.x = 0.20;
            pts.scale.y = 0.20;
            pts.color.r = 0.0f;
            pts.color.g = 0.6f;
            pts.color.b = 1.0f;
            pts.color.a = 1.0f;

            for (const auto & p : forward_points_) {
                pts.points.push_back(make_point(p[0], p[1], 0.08));
            }

            array.markers.push_back(pts);
        }

        // Waypoint objetivo actual
        {
            const auto goal = current_goal();

            visualization_msgs::msg::Marker goal_marker;
            goal_marker.header.frame_id = marker_frame_;
            goal_marker.header.stamp = this->now();
            goal_marker.ns = "mission_goal";
            goal_marker.id = 3;
            goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
            goal_marker.action = visualization_msgs::msg::Marker::ADD;
            goal_marker.pose.position.x = goal[0];
            goal_marker.pose.position.y = goal[1];
            goal_marker.pose.position.z = 0.15;
            goal_marker.pose.orientation.w = 1.0;
            goal_marker.scale.x = 0.35;
            goal_marker.scale.y = 0.35;
            goal_marker.scale.z = 0.35;
            goal_marker.color.r = 1.0f;
            goal_marker.color.g = 0.2f;
            goal_marker.color.b = 0.2f;
            goal_marker.color.a = 1.0f;

            array.markers.push_back(goal_marker);
        }

        marker_pub_->publish(array);
    }

    // ============================================================
    // TIMER
    // ============================================================
    void tick()
    {
        request_control();
        publish_markers();

        if (!control_granted_ || !has_odom_) {
            return;
        }

        const uint64_t ts = now_us();

        if (!trajectory_started_) {
            trajectory_started_ = true;
            reset_pid_state();  // Inicializa last_control_time_ correctamente

            const auto goal = current_goal();

            RCLCPP_INFO(this->get_logger(),
                        "[%s] Trayectoria iniciada. Primer objetivo=(%.2f, %.2f)",
                        drone_ns_.c_str(), goal[0], goal[1]);
        }

        advance_goal_if_reached();
        compute_and_publish_control(ts);
    }

private:
    // Parámetros generales
    std::string drone_ns_;
    std::string ns_prefix_;
    std::string odom_topic_{"odom"};
    std::string marker_frame_{"odom"};

    // Trayectoria
    double height_{5.0};
    double goal_tolerance_{0.30};
    double slowdown_radius_{1.5};

    std::array<double, 2> p1_{0.0, 0.0};
    std::array<double, 2> p2_{3.0, 0.0};
    std::array<double, 2> p3_{3.0, 3.0};
    std::array<double, 2> p4_{0.0, 3.0};

    std::vector<std::array<double, 2>> forward_points_;
    std::size_t goal_index_{0};
    bool reverse_path_{false};

    // PID
    double kp_x_{0.9}, ki_x_{0.0}, kd_x_{0.15};
    double kp_y_{0.9}, ki_y_{0.0}, kd_y_{0.15};
    double kp_z_{0.8}, ki_z_{0.0}, kd_z_{0.10};

    double max_vx_{1.5};
    double max_vy_{1.5};
    double max_vz_{0.8};
    double max_speed_xy_{1.8};

    double integral_limit_x_{2.0};
    double integral_limit_y_{2.0};
    double integral_limit_z_{2.0};

    double control_period_ms_{50.0};

    // Estado odom
    double current_x_{0.0};
    double current_y_{0.0};
    double current_z_{0.0};
    bool has_odom_{false};

    // Estado control
    bool control_requested_{false};
    bool control_granted_{false};
    bool trajectory_started_{false};

    // Estado PID
    double last_control_time_{0.0};
    bool pid_initialized_{false};
    bool first_derivative_tick_{true};   // FIX Bug 2: suprime spike derivativo tras reset

    double ix_{0.0}, iy_{0.0}, iz_{0.0};
    double prev_ex_{0.0}, prev_ey_{0.0}, prev_ez_{0.0};

    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr control_client_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneWaypointMissionClient>());
    rclcpp::shutdown();
    return 0;
}