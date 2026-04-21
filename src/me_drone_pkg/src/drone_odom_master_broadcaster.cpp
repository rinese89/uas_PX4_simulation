#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DroneTFBroadcaster : public rclcpp::Node {
public:
    DroneTFBroadcaster()
    : Node("drone_odom_broadcaster")
    {
        // =========================
        // PARÁMETRO NAMESPACE
        // =========================
        this->declare_parameter<std::string>("drone_ns", "");
        this->get_parameter("drone_ns", drone_ns_);

        ns_prefix_ = drone_ns_.empty() ? "" : "/" + drone_ns_;

        // Frames únicos por dron
        odom_frame_ = drone_ns_.empty() ? "odom" : "odom" + drone_ns_;
        base_frame_ = drone_ns_.empty() ? "base_link" : "base_link_" + drone_ns_;

        RCLCPP_INFO(get_logger(), "Drone namespace: %s", ns_prefix_.c_str());

        // =========================
        // TF broadcaster
        // =========================
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // =========================
        // QoS
        // =========================
        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        // =========================
        // SUB
        // =========================
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            ns_prefix_ + "/fmu/out/vehicle_odometry",
            qos_profile,
            std::bind(&DroneTFBroadcaster::odom_callback, this, std::placeholders::_1));

        // =========================
        // PUB
        // =========================
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            ns_prefix_ + "/odom", 10);
    }

private:
    std::string drone_ns_;
    std::string ns_prefix_;
    std::string odom_frame_;
    std::string base_frame_;

    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id  = base_frame_;

        // Orientación (PX4: w,x,y,z)
        transform.transform.rotation.x = msg->q[1];
        transform.transform.rotation.y = msg->q[2];
        transform.transform.rotation.z = msg->q[3];
        transform.transform.rotation.w = msg->q[0];

        // Posición (NED → ENU parcial)
        transform.transform.translation.x =  msg->position[0];
        transform.transform.translation.y =  msg->position[1];
        transform.transform.translation.z = -msg->position[2];

        tf_broadcaster_->sendTransform(transform);

        // =========================
        // 📡 ODOM MSG
        // =========================
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = transform.header.stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        // Posición
        odom_msg.pose.pose.position.x = transform.transform.translation.x;
        odom_msg.pose.pose.position.y = transform.transform.translation.y;
        odom_msg.pose.pose.position.z = transform.transform.translation.z;

        // Orientación
        odom_msg.pose.pose.orientation = transform.transform.rotation;

        // Velocidad
        odom_msg.twist.twist.linear.x =  msg->velocity[0];
        odom_msg.twist.twist.linear.y =  msg->velocity[1];
        odom_msg.twist.twist.linear.z = -msg->velocity[2];

        odom_msg.twist.twist.angular.x =  msg->angular_velocity[0];
        odom_msg.twist.twist.angular.y =  msg->angular_velocity[1];
        odom_msg.twist.twist.angular.z = -msg->angular_velocity[2];

        odom_pub_->publish(odom_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}