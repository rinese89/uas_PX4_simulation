#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

class ArucoPublisher : public rclcpp::Node
{
public:
    ArucoPublisher()
    : Node("aruco_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("aruco_pose", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&ArucoPublisher::camera_callback, this));

        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        parameters_ = cv::aruco::DetectorParameters::create();

        cap_.open(0, cv::CAP_V4L2);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1920); 
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
	
        intrinsics_ = (cv::Mat_<double>(3, 3) << 1492.044121321748, 0, 980.4590896715763,
                                                 0, 1489.826211867192, 561.5354742023020,
                                                 0, 0, 1);
        distortion_params_ = (cv::Mat_<double>(1, 4) << 0.119776955261496, 0.145125649383559, 0, 0);
    }

private:
    void camera_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo capturar el frame de la cámara");
            return;
        }

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, aruco_dict_, marker_corners, marker_ids, parameters_);

        if (!marker_ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.122, intrinsics_, distortion_params_, rvecs, tvecs);

            for (size_t i = 0; i < marker_ids.size(); ++i) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.frame_id = "camera_link";
                pose_msg.header.stamp = this->now();
                pose_msg.pose.position.x = tvecs[i][0];
                pose_msg.pose.position.y = tvecs[i][1];
                pose_msg.pose.position.z = tvecs[i][2];

                // Convertir rvec a matriz de rotación con OpenCV
                cv::Mat rotation_matrix_cv;
                cv::Rodrigues(rvecs[i], rotation_matrix_cv);

                // Convertir OpenCV Mat a tf2::Matrix3x3
                tf2::Matrix3x3 rotation_matrix(
                    rotation_matrix_cv.at<double>(0, 0), rotation_matrix_cv.at<double>(0, 1), rotation_matrix_cv.at<double>(0, 2),
                    rotation_matrix_cv.at<double>(1, 0), rotation_matrix_cv.at<double>(1, 1), rotation_matrix_cv.at<double>(1, 2),
                    rotation_matrix_cv.at<double>(2, 0), rotation_matrix_cv.at<double>(2, 1), rotation_matrix_cv.at<double>(2, 2)
                );

                double roll, pitch, yaw;
                rotation_matrix.getRPY(roll, pitch, yaw);

                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);

                pose_msg.pose.orientation.x = q.x();
                pose_msg.pose.orientation.y = q.y();
                pose_msg.pose.orientation.z = q.z();
                pose_msg.pose.orientation.w = q.w();

                publisher_->publish(pose_msg);
                
                /*
                cv::Mat rotation_matrix_cv;
                cv::Rodrigues(rvecs[i], rotation_matrix_cv);
                            
                tf2::Matrix3x3 rotation_matrix(
                    rotation_matrix_cv.at<double>(0, 0), rotation_matrix_cv.at<double>(0, 1), rotation_matrix_cv.at<double>(0, 2),
                    rotation_matrix_cv.at<double>(1, 0), rotation_matrix_cv.at<double>(1, 1), rotation_matrix_cv.at<double>(1, 2),
                    rotation_matrix_cv.at<double>(2, 0), rotation_matrix_cv.at<double>(2, 1), rotation_matrix_cv.at<double>(2, 2)
                );
                            
                tf2::Quaternion q;
                rotation_matrix.getRotation(q);  // ← más directo
                            
                pose_msg.pose.orientation.x = q.x();
                pose_msg.pose.orientation.y = q.y();
                pose_msg.pose.orientation.z = q.z();
                pose_msg.pose.orientation.w = q.w();
                */
            }
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> parameters_;
    cv::VideoCapture cap_;
    cv::Mat intrinsics_;
    cv::Mat distortion_params_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
