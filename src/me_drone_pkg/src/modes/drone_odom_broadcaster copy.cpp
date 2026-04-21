#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>

class DroneTFBroadcaster : public rclcpp::Node {
public:
    DroneTFBroadcaster()
    : Node("drone_odom_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos_profile.best_effort();

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&DroneTFBroadcaster::odom_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

private:
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp    = this->get_clock()->now();
        transform.header.frame_id = "odom_drone";
        transform.child_frame_id  = "base_link";

        tf2::Quaternion q_ned(
            msg->q[1],
            msg->q[2],
            msg->q[3],
            msg->q[0]);

        // NED - ENU
        tf2::Quaternion q_rot_z;
        q_rot_z.setRPY(0, 0, M_PI/2);

        tf2::Quaternion q_rot_x;
        q_rot_x.setRPY(M_PI, 0, 0);

        tf2::Quaternion q_enu = (q_rot_z * q_rot_x) * q_ned;

        // FIX
        tf2::Quaternion q_body_fix;
        q_body_fix.setRPY(M_PI, 0, 0);

        q_enu = q_body_fix * q_enu;

        q_enu.normalize();

        transform.transform.rotation.x = q_enu.x();
        transform.transform.rotation.y = q_enu.y();
        transform.transform.rotation.z = q_enu.z();
        transform.transform.rotation.w = q_enu.w();

        transform.transform.translation.x =  msg->position[1];
        transform.transform.translation.y =  msg->position[0];
        transform.transform.translation.z = -msg->position[2];

        tf_broadcaster_->sendTransform(transform);

        // Odometría topic
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = transform.header.stamp;
        odom_msg.header.frame_id = "odom_drone";
        odom_msg.child_frame_id = "base_link";

        // Posición
        odom_msg.pose.pose.position.x = transform.transform.translation.x;
        odom_msg.pose.pose.position.y = transform.transform.translation.y;
        odom_msg.pose.pose.position.z = transform.transform.translation.z;

        // Orientación
        odom_msg.pose.pose.orientation = transform.transform.rotation;

        // Velocidad
        odom_msg.twist.twist.linear.x =  msg->velocity[1];
        odom_msg.twist.twist.linear.y =  msg->velocity[0];
        odom_msg.twist.twist.linear.z = -msg->velocity[2];

        odom_msg.twist.twist.angular.x =  msg->angular_velocity[1];
        odom_msg.twist.twist.angular.y =  msg->angular_velocity[0];
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
