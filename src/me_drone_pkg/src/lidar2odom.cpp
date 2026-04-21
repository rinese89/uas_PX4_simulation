#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class LidarToMap : public rclcpp::Node
{
public:
  LidarToMap()
  : Node("lidar_to_map")
  {
    this->declare_parameter<std::string>("source_frame", "x500_mono_cam_0/lidar_link/lidar_2d");
    this->declare_parameter<std::string>("target_frame", "map");
    this->declare_parameter<std::string>("input_topic",  "/scan");
    this->declare_parameter<std::string>("output_topic", "/scan_map");

    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    std::string input_topic  = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, 10,
      std::bind(&LidarToMap::scanCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(),
      "lidar_to_map: %s [%s] -> %s [%s]",
      input_topic.c_str(),  source_frame_.c_str(),
      output_topic.c_str(), target_frame_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 1. Obtener transform sensor -> map en el instante del mensaje
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_,
        source_frame_,
        msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF no disponible [%s -> %s]: %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
      return;
    }

    // 2. Proyectar LaserScan -> PointCloud2 en el frame del sensor
    sensor_msgs::msg::PointCloud2 cloud_sensor;
    projector_.projectLaser(*msg, cloud_sensor);

    // 3. Transformar la nube al frame map
    sensor_msgs::msg::PointCloud2 cloud_map;
    tf2::doTransform(cloud_sensor, cloud_map, transform);

    // 4. Publicar
    cloud_map.header.stamp    = msg->header.stamp;
    cloud_map.header.frame_id = target_frame_;
    pub_->publish(cloud_map);
  }

  std::string source_frame_;
  std::string target_frame_;

  laser_geometry::LaserProjection             projector_;
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarToMap>());
  rclcpp::shutdown();
  return 0;
}