#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>

class DroneTFBroadcaster : public rclcpp::Node {
public:
    DroneTFBroadcaster()
    : Node("drone_odom_broadcaster")
    {
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<int>("drone_id", 0);

        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("drone_id", drone_id_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        odom_frame_ = "odom";
        base_frame_ = drone_ns_.empty() ? "base_link" : "base_link_" + drone_ns_;

        x_offset_ = static_cast<double>(drone_id_);

        RCLCPP_INFO(
            get_logger(),
            "Drone namespace: %s | drone_id: %d | x_offset: %.2f",
            ns_prefix_.c_str(), drone_id_, x_offset_);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        rclcpp::QoS qos_profile(
            rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            ns_prefix_ + "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&DroneTFBroadcaster::odom_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            ns_prefix_ + "/odom", 10);

        reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
            ns_prefix_ + "/reset_odom_srv",
            std::bind(
                &DroneTFBroadcaster::reset_service_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
    }

private:
    using VehicleOdometry = px4_msgs::msg::VehicleOdometry;
    namespace ft = px4_ros_com::frame_transforms;

    static bool finite3(const float v[3])
    {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    }

    static bool finite4(const float q[4])
    {
        return std::isfinite(q[0]) && std::isfinite(q[1]) &&
               std::isfinite(q[2]) && std::isfinite(q[3]);
    }

    void odom_callback(const VehicleOdometry::SharedPtr msg)
    {
        current_odom_msg_ = std::make_shared<VehicleOdometry>(*msg);

        if (!finite3(msg->position) || !finite4(msg->q)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "VehicleOdometry con position o quaternion no finitos; se ignora.");
            return;
        }

        // ------------------------------------------------------------------
        // 1) Posición
        // ------------------------------------------------------------------
        Eigen::Vector3d pos_ros = Eigen::Vector3d::Zero();

        if (msg->pose_frame == VehicleOdometry::POSE_FRAME_NED) {
            Eigen::Vector3d pos_ned(
                static_cast<double>(msg->position[0]) - origin_pose_x_,
                static_cast<double>(msg->position[1]) - origin_pose_y_,
                static_cast<double>(msg->position[2]) - origin_pose_z_);

            pos_ros = ft::ned_to_enu_local_frame(pos_ned);
        } else if (msg->pose_frame == VehicleOdometry::POSE_FRAME_FRD) {
            // PX4 define también POSE_FRAME_FRD como world-fixed con heading arbitrario.
            // Aquí no hacemos una falsa conversión a ENU: mantenemos ese frame local
            // y solo aplicamos el reset de origen.
            pos_ros = Eigen::Vector3d(
                static_cast<double>(msg->position[0]) - origin_pose_x_,
                static_cast<double>(msg->position[1]) - origin_pose_y_,
                static_cast<double>(msg->position[2]) - origin_pose_z_);
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_frame desconocido: %u. Se ignora mensaje.",
                static_cast<unsigned>(msg->pose_frame));
            return;
        }

        // ------------------------------------------------------------------
        // 2) Orientación
        // PX4 q: aircraft(FRD) -> reference frame
        // ROS q: base_link(FLU) -> ENU
        // ------------------------------------------------------------------
        Eigen::Quaterniond q_px4(
            static_cast<double>(msg->q[0]),
            static_cast<double>(msg->q[1]),
            static_cast<double>(msg->q[2]),
            static_cast<double>(msg->q[3]));

        Eigen::Quaterniond q_ros = Eigen::Quaterniond::Identity();

        if (msg->pose_frame == VehicleOdometry::POSE_FRAME_NED) {
            q_ros = ft::px4_to_ros_orientation(q_px4);
        } else if (msg->pose_frame == VehicleOdometry::POSE_FRAME_FRD) {
            // No existe una conversión directa "px4_to_ros_orientation" para world FRD.
            // Solo convertimos el cuerpo aircraft(FRD) -> base_link(FLU),
            // manteniendo el frame mundo local tal como llega.
            q_ros = ft::aircraft_to_baselink_orientation(q_px4);
        }

        q_ros.normalize();

        // ------------------------------------------------------------------
        // 3) Velocidad lineal
        // ------------------------------------------------------------------
        Eigen::Vector3d vel_ros = Eigen::Vector3d::Zero();

        if (finite3(msg->velocity)) {
            Eigen::Vector3d vel_in(
                static_cast<double>(msg->velocity[0]),
                static_cast<double>(msg->velocity[1]),
                static_cast<double>(msg->velocity[2]));

            switch (msg->velocity_frame) {
                case VehicleOdometry::VELOCITY_FRAME_NED:
                    vel_ros = ft::ned_to_enu_local_frame(vel_in);
                    break;

                case VehicleOdometry::VELOCITY_FRAME_FRD:
                case VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
                    vel_ros = ft::aircraft_to_baselink_body_frame(vel_in);
                    break;

                default:
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "velocity_frame desconocido: %u. Se publica velocidad cero.",
                        static_cast<unsigned>(msg->velocity_frame));
                    break;
            }
        }

        // ------------------------------------------------------------------
        // 4) Velocidad angular
        // PX4 la define en body-fixed FRD.
        // ROS base_link usa FLU.
        // ------------------------------------------------------------------
        Eigen::Vector3d ang_frd(
            static_cast<double>(msg->angular_velocity[0]),
            static_cast<double>(msg->angular_velocity[1]),
            static_cast<double>(msg->angular_velocity[2]));

        Eigen::Vector3d ang_ros = ft::aircraft_to_baselink_body_frame(ang_frd);

        // ------------------------------------------------------------------
        // 5) TF odom -> base_link
        // ------------------------------------------------------------------
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = base_frame_;

        transform.transform.translation.x = pos_ros.x() + x_offset_;
        transform.transform.translation.y = pos_ros.y();
        transform.transform.translation.z = pos_ros.z();

        transform.transform.rotation.w = q_ros.w();
        transform.transform.rotation.x = q_ros.x();
        transform.transform.rotation.y = q_ros.y();
        transform.transform.rotation.z = q_ros.z();

        tf_broadcaster_->sendTransform(transform);

        // ------------------------------------------------------------------
        // 6) nav_msgs/Odometry
        // ------------------------------------------------------------------
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = transform.header.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        odom_msg.pose.pose.position.x = transform.transform.translation.x;
        odom_msg.pose.pose.position.y = transform.transform.translation.y;
        odom_msg.pose.pose.position.z = transform.transform.translation.z;

        odom_msg.pose.pose.orientation.w = transform.transform.rotation.w;
        odom_msg.pose.pose.orientation.x = transform.transform.rotation.x;
        odom_msg.pose.pose.orientation.y = transform.transform.rotation.y;
        odom_msg.pose.pose.orientation.z = transform.transform.rotation.z;

        odom_msg.twist.twist.linear.x = vel_ros.x();
        odom_msg.twist.twist.linear.y = vel_ros.y();
        odom_msg.twist.twist.linear.z = vel_ros.z();

        odom_msg.twist.twist.angular.x = ang_ros.x();
        odom_msg.twist.twist.angular.y = ang_ros.y();
        odom_msg.twist.twist.angular.z = ang_ros.z();

        odom_pub_->publish(odom_msg);
    }

    void reset_service_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!current_odom_msg_) {
            res->success = false;
            res->message = "No hay odometria disponible todavia";
            RCLCPP_WARN(
                get_logger(),
                "[%s] Reset pedido sin haber recibido odometria",
                drone_ns_.c_str());
            return;
        }

        set_origin_from_msg(current_odom_msg_);

        res->success = true;
        res->message = "Odometria reseteada";

        RCLCPP_INFO(
            get_logger(),
            "[%s] Reset de odometría aplicado. Nuevo origen en pose_frame=%u: "
            "(%.3f, %.3f, %.3f)",
            drone_ns_.c_str(),
            static_cast<unsigned>(origin_pose_frame_),
            origin_pose_x_, origin_pose_y_, origin_pose_z_);
    }

    void set_origin_from_msg(const VehicleOdometry::SharedPtr msg)
    {
        if (!msg || !finite3(msg->position)) {
            RCLCPP_WARN(
                get_logger(),
                "Mensaje de odometria nulo o no finito al fijar origen");
            return;
        }

        origin_pose_x_ = static_cast<double>(msg->position[0]);
        origin_pose_y_ = static_cast<double>(msg->position[1]);
        origin_pose_z_ = static_cast<double>(msg->position[2]);
        origin_pose_frame_ = msg->pose_frame;
    }

    std::string drone_ns_;
    std::string ns_prefix_;
    std::string odom_frame_;
    std::string base_frame_;

    int drone_id_{0};
    double x_offset_{0.0};

    double origin_pose_x_{0.0};
    double origin_pose_y_{0.0};
    double origin_pose_z_{0.0};
    uint8_t origin_pose_frame_{VehicleOdometry::POSE_FRAME_UNKNOWN};

    VehicleOdometry::SharedPtr current_odom_msg_;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}