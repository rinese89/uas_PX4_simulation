#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <memory>
#include <string>

// =============================================================================
// PX4 -> ROS frame conventions
//
// PX4 world frames:
//   - NED: North East Down
//   - FRD: Forward Right Down (world-fixed, arbitrary heading reference)
//
// PX4 body frame:
//   - FRD
//
// ROS world frame:
//   - ENU
//
// ROS body frame:
//   - FLU (base_link)
//
// We therefore need:
//
//   World: NED -> ENU
//   Body : FRD -> FLU
//
// For orientation, PX4 VehicleOdometry.q is documented as the quaternion that
// rotates from the FRD body frame to the reference frame.
//
// This implementation converts that attitude to a ROS-compatible quaternion
// for odom(ENU) -> base_link(FLU).
// =============================================================================

namespace
{

inline bool is_finite3(const std::array<float, 3> &v)
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline bool is_finite4(const std::array<float, 4> &q)
{
    return std::isfinite(q[0]) && std::isfinite(q[1]) &&
           std::isfinite(q[2]) && std::isfinite(q[3]);
}

// -----------------------------------------------------------------------------
// Static transforms
// -----------------------------------------------------------------------------

// NED -> ENU as rotation matrix:
// [x_enu]   [0 1  0][x_ned]
// [y_enu] = [1 0  0][y_ned]
// [z_enu]   [0 0 -1][z_ned]
inline Eigen::Matrix3d ned_to_enu_rotation()
{
    Eigen::Matrix3d R;
    R << 0.0, 1.0,  0.0,
         1.0, 0.0,  0.0,
         0.0, 0.0, -1.0;
    return R;
}

// FRD -> FLU:
// x forward stays x
// y right   -> y left = -y
// z down    -> z up   = -z
inline Eigen::Matrix3d frd_to_flu_rotation()
{
    Eigen::Matrix3d R;
    R << 1.0,  0.0,  0.0,
         0.0, -1.0,  0.0,
         0.0,  0.0, -1.0;
    return R;
}

inline Eigen::Vector3d ned_to_enu_vector(const Eigen::Vector3d &v_ned)
{
    static const Eigen::Matrix3d R = ned_to_enu_rotation();
    return R * v_ned;
}

inline Eigen::Vector3d frd_to_flu_vector(const Eigen::Vector3d &v_frd)
{
    static const Eigen::Matrix3d R = frd_to_flu_rotation();
    return R * v_frd;
}

// Convert PX4 attitude quaternion to ROS quaternion.
//
// PX4 q = rotation from body(FRD) -> reference frame.
// Let R_ref_body_frd be that rotation matrix.
//
// ROS nav_msgs/Odometry pose expects orientation of child frame (base_link)
// with respect to parent frame (odom). In matrix form we want:
//
//   R_odom_baselink_flu
//
// If reference is NED:
//   R_enu_flu = R_ned_to_enu * R_ned_frd_body?  careful:
//   q_px4 gives body->ref, so matrix is R_ref_body.
//   We need parent->child? For ROS quaternion in pose, we use child wrt parent,
//   which is conventionally the rotation matrix R_parent_child.
//   Thus:
//
//   R_ned_body_frd = inverse(R_body_to_ned) ??? Actually q is body->ref,
//   so matrix from body to ref is R_ref_body.
//
//   For ROS we want R_enu_baselink.
//
//   Using frame basis change:
//   v_enu = R_ned_to_enu * v_ned
//   v_flu = R_frd_to_flu * v_frd
//
//   v_enu = R_ned_to_enu * R_ref_body * v_frd
//         = R_ned_to_enu * R_ref_body * R_flu_to_frd * v_flu
//
// therefore:
//   R_enu_flu = R_ned_to_enu * R_ref_body * R_flu_to_frd
//
// and R_flu_to_frd = inverse(R_frd_to_flu) = same here because diagonal.
//
// For reference frame FRD(world-fixed arbitrary heading), replace NED->ENU with
// identity on world semantics? There is no exact ENU equivalent because FRD
// world frame is arbitrary heading referenced. We keep it as ROS odom-like local
// frame by only converting body FRD->FLU and preserving the world frame axes as
// already local world-fixed. In practice, if pose_frame==FRD, we publish in odom
// semantics but this is not georeferenced ENU.
// -----------------------------------------------------------------------------
inline Eigen::Quaterniond px4_q_to_ros_q(
    const std::array<float, 4> &q_px4,
    uint8_t pose_frame)
{
    const Eigen::Quaterniond q_body_to_ref_px4(
        static_cast<double>(q_px4[0]),
        static_cast<double>(q_px4[1]),
        static_cast<double>(q_px4[2]),
        static_cast<double>(q_px4[3]));

    Eigen::Matrix3d R_ref_body = q_body_to_ref_px4.normalized().toRotationMatrix();

    const Eigen::Matrix3d R_frd_to_flu = frd_to_flu_rotation();
    const Eigen::Matrix3d R_flu_to_frd = R_frd_to_flu.transpose();

    Eigen::Matrix3d R_ros_parent_child;

    if (pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
        const Eigen::Matrix3d R_ned_to_enu = ned_to_enu_rotation();
        R_ros_parent_child = R_ned_to_enu * R_ref_body * R_flu_to_frd;
    } else if (pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD) {
        // World-fixed FRD with arbitrary heading reference.
        // Preserve that local/world frame and only convert body FRD -> FLU.
        R_ros_parent_child = R_ref_body * R_flu_to_frd;
    } else {
        return Eigen::Quaterniond::Identity();
    }

    Eigen::Quaterniond q_ros(R_ros_parent_child);
    q_ros.normalize();
    return q_ros;
}

}  // namespace

class DroneTFBroadcaster : public rclcpp::Node
{
public:
    DroneTFBroadcaster()
    : Node("drone_odom_broadcaster")
    {
        this->declare_parameter<std::string>("drone_ns", "");
        this->declare_parameter<int>("drone_id", 0);

        this->get_parameter("drone_ns", drone_ns_);
        this->get_parameter("drone_id", drone_id_);

        ns_prefix_  = drone_ns_.empty() ? "" : "/" + drone_ns_;
        odom_frame_ = "odom";
        base_frame_ = drone_ns_.empty() ? "base_link" : "base_link_" + drone_ns_;
        x_offset_   = static_cast<double>(drone_id_);

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
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        current_odom_msg_ = std::make_shared<px4_msgs::msg::VehicleOdometry>(*msg);

        if (!is_finite3(msg->position) || !is_finite4(msg->q)) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "VehicleOdometry contiene position/q no finitos. Se ignora mensaje.");
            return;
        }

        // ------------------------------------------------------------------
        // 1) Position
        // ------------------------------------------------------------------
        Eigen::Vector3d pos_ros = Eigen::Vector3d::Zero();

        if (msg->pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
            const Eigen::Vector3d pos_ned(
                static_cast<double>(msg->position[0]) - origin_pose_x_,
                static_cast<double>(msg->position[1]) - origin_pose_y_,
                static_cast<double>(msg->position[2]) - origin_pose_z_);
            pos_ros = ned_to_enu_vector(pos_ned);
        } else if (msg->pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD) {
            // Local/world-fixed FRD. Keep as local odom-like frame and only reset origin.
            pos_ros.x() = static_cast<double>(msg->position[0]) - origin_pose_x_;
            pos_ros.y() = static_cast<double>(msg->position[1]) - origin_pose_y_;
            pos_ros.z() = static_cast<double>(msg->position[2]) - origin_pose_z_;
        } else {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "pose_frame desconocido: %u. Se ignora mensaje.",
                static_cast<unsigned>(msg->pose_frame));
            return;
        }

        // ------------------------------------------------------------------
        // 2) Orientation
        // ------------------------------------------------------------------
        const Eigen::Quaterniond q_ros = px4_q_to_ros_q(msg->q, msg->pose_frame);

        // ------------------------------------------------------------------
        // 3) Linear velocity
        // ------------------------------------------------------------------
        Eigen::Vector3d vel_ros = Eigen::Vector3d::Zero();
        if (is_finite3(msg->velocity)) {
            const Eigen::Vector3d v_in(
                static_cast<double>(msg->velocity[0]),
                static_cast<double>(msg->velocity[1]),
                static_cast<double>(msg->velocity[2]));

            switch (msg->velocity_frame) {
                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED:
                    vel_ros = ned_to_enu_vector(v_in);
                    break;

                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD:
                case px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD:
                    vel_ros = frd_to_flu_vector(v_in);
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
        // 4) Angular velocity
        // PX4 defines angular_velocity in body-fixed FRD.
        // ROS base_link is FLU.
        // ------------------------------------------------------------------
        Eigen::Vector3d ang_ros = Eigen::Vector3d::Zero();
        {
            const Eigen::Vector3d ang_frd(
                static_cast<double>(msg->angular_velocity[0]),
                static_cast<double>(msg->angular_velocity[1]),
                static_cast<double>(msg->angular_velocity[2]));
            ang_ros = frd_to_flu_vector(ang_frd);
        }

        // ------------------------------------------------------------------
        // 5) Publish TF odom -> base_link
        // ------------------------------------------------------------------
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp    = this->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id  = base_frame_;

        transform.transform.translation.x = pos_ros.x() + x_offset_;
        transform.transform.translation.y = pos_ros.y();
        transform.transform.translation.z = pos_ros.z();

        transform.transform.rotation.w = q_ros.w();
        transform.transform.rotation.x = q_ros.x();
        transform.transform.rotation.y = q_ros.y();
        transform.transform.rotation.z = q_ros.z();

        tf_broadcaster_->sendTransform(transform);

        // ------------------------------------------------------------------
        // 6) Publish nav_msgs/Odometry
        // ------------------------------------------------------------------
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp    = transform.header.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id  = base_frame_;

        odom_msg.pose.pose.position.x    = transform.transform.translation.x;
        odom_msg.pose.pose.position.y    = transform.transform.translation.y;
        odom_msg.pose.pose.position.z    = transform.transform.translation.z;
        odom_msg.pose.pose.orientation.w = transform.transform.rotation.w;
        odom_msg.pose.pose.orientation.x = transform.transform.rotation.x;
        odom_msg.pose.pose.orientation.y = transform.transform.rotation.y;
        odom_msg.pose.pose.orientation.z = transform.transform.rotation.z;

        odom_msg.twist.twist.linear.x  = vel_ros.x();
        odom_msg.twist.twist.linear.y  = vel_ros.y();
        odom_msg.twist.twist.linear.z  = vel_ros.z();

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
            "[%s] Reset de odometría aplicado. Nuevo origen pose frame: "
            "(%.3f, %.3f, %.3f)",
            drone_ns_.c_str(),
            origin_pose_x_, origin_pose_y_, origin_pose_z_);
    }

    void set_origin_from_msg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        if (!msg || !is_finite3(msg->position)) {
            RCLCPP_WARN(get_logger(), "Mensaje de odometria nulo o no finito al fijar origen");
            return;
        }

        // Guardamos el origen en el frame nativo del pose_frame recibido.
        origin_pose_x_ = static_cast<double>(msg->position[0]);
        origin_pose_y_ = static_cast<double>(msg->position[1]);
        origin_pose_z_ = static_cast<double>(msg->position[2]);
        origin_pose_frame_ = msg->pose_frame;
    }

private:
    std::string drone_ns_;
    std::string ns_prefix_;
    std::string odom_frame_;
    std::string base_frame_;

    int    drone_id_{0};
    double x_offset_{0.0};

    // Origen guardado en el frame nativo de pose.
    double  origin_pose_x_{0.0};
    double  origin_pose_y_{0.0};
    double  origin_pose_z_{0.0};
    uint8_t origin_pose_frame_{px4_msgs::msg::VehicleOdometry::POSE_FRAME_UNKNOWN};

    px4_msgs::msg::VehicleOdometry::SharedPtr current_odom_msg_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
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