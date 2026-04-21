/**
 * orb_matcher_node.cpp
 *
 * Nodo ROS2 – Fase 2: ORB matching + Triangulación → profundidad por terminal
 *
 * Flujo de transformadas (sin latencia de TF en el bucle principal):
 *
 *   1. Una sola vez al arrancar:
 *        lookupTransform(base_link → camera)  →  T_cam_base_  (TF fijo)
 *
 *   2. Cada mensaje /odom nos da:
 *        pose de base_link en odom            →  T_base_odom  (variable)
 *
 *   3. Composición en cada frame:
 *        T_cam_odom = T_base_odom * T_cam_base_               (sin TF lookup)
 *
 * Suscripciones:
 *   /camera/image_raw      (sensor_msgs/Image)
 *   /camera/camera_info    (sensor_msgs/CameraInfo)
 *   /odom                  (nav_msgs/Odometry)
 *
 * Salida:
 *   - Ventana OpenCV con matches ORB
 *   - Terminal: pose de la cámara en odom + tabla de puntos 3D triangulados
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────────
//  Defaults
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int    ORB_N_FEATURES  = 1000;
static constexpr double LOWE_RATIO      = 0.75;
static constexpr int    MIN_MATCHES     = 15;
static constexpr double SKIP_DIST_M     = 0.50;
static constexpr double SKIP_ANGLE_RAD  = 0.20;
static const std::string WINDOW_NAME    = "ORB Matches";

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: TransformStamped → Eigen::Isometry3d
// ─────────────────────────────────────────────────────────────────────────────
static Eigen::Isometry3d tfToEigen(const geometry_msgs::msg::TransformStamped & tf)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << tf.transform.translation.x,
                     tf.transform.translation.y,
                     tf.transform.translation.z;
  Eigen::Quaterniond q(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);
  if (q.w() < 0.0) q.coeffs() = -q.coeffs();
  T.linear() = q.normalized().toRotationMatrix();
  return T;
}

// Helper: geometry_msgs::Pose → Eigen::Isometry3d
static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose & p)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << p.position.x, p.position.y, p.position.z;
  Eigen::Quaterniond q(
    p.orientation.w,
    p.orientation.x,
    p.orientation.y,
    p.orientation.z);
  // Forzar hemisferio w>=0: q y -q son la misma rotacion pero al componer
  // transformadas entre iteraciones el signo inconsistente causa flip de orientacion
  if (q.w() < 0.0) q.coeffs() = -q.coeffs();
  T.linear() = q.normalized().toRotationMatrix();
  return T;
}

// Helper: T_cam_in_odom → matriz de proyección P = K * [R|t]  (odom→camera)
static cv::Mat makeProjection(const Eigen::Isometry3d & T_cam_in_odom,
                              const cv::Mat            & K)
{
  Eigen::Isometry3d T_odom_to_cam = T_cam_in_odom.inverse();
  cv::Mat Rt(3, 4, CV_64F);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      Rt.at<double>(r, c) = T_odom_to_cam.linear()(r, c);
  Rt.at<double>(0, 3) = T_odom_to_cam.translation().x();
  Rt.at<double>(1, 3) = T_odom_to_cam.translation().y();
  Rt.at<double>(2, 3) = T_odom_to_cam.translation().z();
  return K * Rt;
}

// ─────────────────────────────────────────────────────────────────────────────
class OrbMatcherNode : public rclcpp::Node
// ─────────────────────────────────────────────────────────────────────────────
{
public:
  OrbMatcherNode()
  : Node("orb_matcher_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // ── Parámetros ────────────────────────────────────────────────────────
    this->declare_parameter("orb_n_features",   ORB_N_FEATURES);
    this->declare_parameter("lowe_ratio",        LOWE_RATIO);
    this->declare_parameter("min_matches",       MIN_MATCHES);
    this->declare_parameter("skip_dist_m",       SKIP_DIST_M);
    this->declare_parameter("skip_angle_rad",    SKIP_ANGLE_RAD);
    this->declare_parameter("show_window",       true);
    this->declare_parameter("camera_frame",      std::string("camera_link"));
    this->declare_parameter("base_link_frame",   std::string("base_link"));
    this->declare_parameter("odom_frame",        std::string("odom_drone"));
    this->declare_parameter("verbose_points",    true);

    int n_feat       = this->get_parameter("orb_n_features").as_int();
    lowe_ratio_      = this->get_parameter("lowe_ratio").as_double();
    min_matches_     = this->get_parameter("min_matches").as_int();
    skip_dist_       = this->get_parameter("skip_dist_m").as_double();
    skip_angle_      = this->get_parameter("skip_angle_rad").as_double();
    show_window_     = this->get_parameter("show_window").as_bool();
    camera_frame_    = this->get_parameter("camera_frame").as_string();
    base_link_frame_ = this->get_parameter("base_link_frame").as_string();
    odom_frame_      = this->get_parameter("odom_frame").as_string();
    verbose_pts_     = this->get_parameter("verbose_points").as_bool();

    // ── ORB + BFMatcher ───────────────────────────────────────────────────
    orb_     = cv::ORB::create(n_feat);
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);

    // ── Suscripciones ─────────────────────────────────────────────────────
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image", 10,
      std::bind(&OrbMatcherNode::imageCallback, this, std::placeholders::_1));

    sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", rclcpp::QoS(1).reliable(),
      std::bind(&OrbMatcherNode::cameraInfoCallback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OrbMatcherNode::odomCallback, this, std::placeholders::_1));

    // ── Publicador PointCloud2 ─────────────────────────────────────────
    pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/point_cloud", rclcpp::QoS(10).reliable());

    // ── Ventana OpenCV ────────────────────────────────────────────────────
    if (show_window_) {
      cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
      cv::resizeWindow(WINDOW_NAME, 1280, 480);
    }

    RCLCPP_INFO(this->get_logger(),
      "OrbMatcherNode Fase 2 | features=%d | skip_dist=%.2f m | "
      "cam='%s' | base='%s' | odom='%s'",
      n_feat, skip_dist_,
      camera_frame_.c_str(), base_link_frame_.c_str(), odom_frame_.c_str());
  }

  ~OrbMatcherNode() override
  {
    if (show_window_) cv::destroyAllWindows();
  }

private:
  // ─── /camera/camera_info ──────────────────────────────────────────────────
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (camera_info_received_) return;

    K_ = cv::Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        K_.at<double>(r, c) = msg->k[r * 3 + c];

    dist_coeffs_ = cv::Mat(1, static_cast<int>(msg->d.size()), CV_64F);
    for (size_t i = 0; i < msg->d.size(); ++i)
      dist_coeffs_.at<double>(0, static_cast<int>(i)) = msg->d[i];

    if (!msg->header.frame_id.empty())
      camera_frame_ = msg->header.frame_id;

    camera_info_received_ = true;

    RCLCPP_INFO(this->get_logger(),
      "CameraInfo OK | fx=%.1f fy=%.1f cx=%.1f cy=%.1f | frame='%s'",
      K_.at<double>(0,0), K_.at<double>(1,1),
      K_.at<double>(0,2), K_.at<double>(1,2),
      camera_frame_.c_str());
  }

  // ─── /odom ────────────────────────────────────────────────────────────────
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    odom_received_ = true;
  }

  // ─── /camera/image_raw ────────────────────────────────────────────────────
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    // Esperar calibración de cámara
    if (!camera_info_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Esperando /camera/camera_info...");
      return;
    }

    // ── Leer TF fijo base_link→camera UNA SOLA VEZ ───────────────────────
    if (!static_tf_ready_) {
      if (!readStaticTF()) return;
    }

    // Esperar primer mensaje de odometría
    if (!odom_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Esperando /odom...");
      return;
    }

    // Convertir imagen a escala de grises
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    cv::Mat frame_gray = cv_ptr->image;

    // ── Primer fotograma ──────────────────────────────────────────────────
    if (prev_frame_.empty()) {
      frame_gray.copyTo(prev_frame_);
      prev_pose_  = current_pose_;
      prev_stamp_ = msg->header.stamp;
      // Guardar pose de la cámara en odom para este instante
      T_prev_cam_ = computeCameraInOdom(current_pose_);
      printCameraPose(T_prev_cam_, "Primer fotograma almacenado");
      return;
    }

    // ── Filtro de movimiento (usando odometría, sin TF) ───────────────────
    if (!hasMoved(prev_pose_, current_pose_)) return;

    // ── Pose actual de la cámara en odom (composición, sin lookupTransform) ─
    //    T_cam_odom = T_base_odom * T_cam_base_
    Eigen::Isometry3d T_curr_cam = computeCameraInOdom(current_pose_);
    Eigen::Isometry3d T_prev_cam = T_prev_cam_;  // guardado en la iteración anterior

    // Baseline real entre las dos posiciones de la cámara
    double baseline = (T_curr_cam.translation() - T_prev_cam.translation()).norm();
    if (baseline < 1e-4) {
      RCLCPP_WARN(this->get_logger(),
        "Baseline demasiado pequeno (%.4f m), skip.", baseline);
      updatePrev(frame_gray, current_pose_, msg->header.stamp, T_curr_cam);
      return;
    }

    printCameraPose(T_curr_cam, "Frame actual");

    // ── Detección ORB ─────────────────────────────────────────────────────
    std::vector<cv::KeyPoint> kp_prev, kp_curr;
    cv::Mat desc_prev, desc_curr;
    orb_->detectAndCompute(prev_frame_, cv::noArray(), kp_prev, desc_prev);
    orb_->detectAndCompute(frame_gray,  cv::noArray(), kp_curr, desc_curr);

    if (desc_prev.empty() || desc_curr.empty()) {
      updatePrev(frame_gray, current_pose_, msg->header.stamp, T_curr_cam);
      return;
    }

    // ── Matching con ratio test de Lowe ───────────────────────────────────
    std::vector<std::vector<cv::DMatch>> knn;
    matcher_->knnMatch(desc_prev, desc_curr, knn, 2);

    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for (auto & m : knn) {
      if (m.size() < 2) continue;
      if (m[0].distance < lowe_ratio_ * m[1].distance)
        good.push_back(m[0]);
    }

    RCLCPP_INFO(this->get_logger(),
      "KP prev=%zu curr=%zu | matches=%zu | baseline=%.3f m",
      kp_prev.size(), kp_curr.size(), good.size(), baseline);

    if (static_cast<int>(good.size()) < min_matches_) {
      RCLCPP_WARN(this->get_logger(), "Pocos matches (%zu < %d), skip.",
        good.size(), min_matches_);
      updatePrev(frame_gray, current_pose_, msg->header.stamp, T_curr_cam);
      return;
    }

    // ── Visualización matches ─────────────────────────────────────────────
    if (show_window_) {
      cv::Mat img_matches;
      cv::drawMatches(prev_frame_, kp_prev, frame_gray, kp_curr,
                      good, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                      {}, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
      std::string info = "Matches: " + std::to_string(good.size())
                       + "  Baseline: " + std::to_string(baseline).substr(0,5) + " m";
      cv::putText(img_matches, info, {10, 25},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
      //cv::imshow(WINDOW_NAME, img_matches);
      //cv::waitKey(1);
    }

    // ── Triangulación + impresión ─────────────────────────────────────────
    triangulateAndPublish(kp_prev, kp_curr, good, T_prev_cam, T_curr_cam, baseline, msg->header.stamp);

    updatePrev(frame_gray, current_pose_, msg->header.stamp, T_curr_cam);
  }

  // ─── Lee el TF fijo base_link → camera (una sola vez) ────────────────────
  bool readStaticTF()
  {
    try {
      // Pedimos el último TF disponible (es estático, siempre el mismo)
      auto tf = tf_buffer_.lookupTransform(
        base_link_frame_, camera_frame_,
        tf2::TimePointZero);
      T_cam_base_    = tfToEigen(tf);
      static_tf_ready_ = true;

      RCLCPP_INFO(this->get_logger(),
        "TF fijo '%s'→'%s' leido OK: t=[%.4f, %.4f, %.4f]",
        base_link_frame_.c_str(), camera_frame_.c_str(),
        T_cam_base_.translation().x(),
        T_cam_base_.translation().y(),
        T_cam_base_.translation().z());
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "TF fijo '%s'→'%s' no disponible aun: %s",
        base_link_frame_.c_str(), camera_frame_.c_str(), ex.what());
      return false;
    }
  }

  // ─── Composición: pose de la cámara en odom sin lookupTransform ──────────
  //     T_cam_odom = T_base_odom  *  T_cam_base_
  //                  (de /odom)      (TF fijo leído una vez)
  Eigen::Isometry3d computeCameraInOdom(const geometry_msgs::msg::Pose & base_pose) const
  {
    return poseToEigen(base_pose) * T_cam_base_;
  }

  // ─── Imprime la pose de la cámara en el frame odom ───────────────────────
  void printCameraPose(const Eigen::Isometry3d & T, const std::string & label) const
  {
    Eigen::Vector3d    t = T.translation();
    // Extraer quaternion desde la matriz de rotacion y forzar w>=0
    // (Eigen::Isometry3d almacena R como matriz 3x3; al reconvertir a
    //  quaternion el signo de w es indeterminado)
    Eigen::Quaterniond q(T.rotation());
    if (q.w() < 0.0) q.coeffs() = -q.coeffs();
    // Extraer RPY con atan2 desde la matriz de rotacion (sin singularidades de eulerAngles)
    // Convencion ZYX: yaw=Z, pitch=Y, roll=X  (estandar ROS)
    const auto & R = T.rotation();
    double roll  = std::atan2( R(2,1), R(2,2));
    double pitch = std::atan2(-R(2,0), std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));
    double yaw   = std::atan2( R(1,0), R(0,0));
    auto r2d = [](double r){ return r * 180.0 / M_PI; };
    std::cout << std::fixed << std::setprecision(4)
      << "\n[camera en '" << odom_frame_ << "'] " << label << "\n"
      << "  posicion   : x=" << std::setw(8) << t.x()
      << "  y=" << std::setw(8) << t.y()
      << "  z=" << std::setw(8) << t.z() << "\n"
      << "  quaternion : qx=" << std::setw(8) << q.x()
      << "  qy=" << std::setw(8) << q.y()
      << "  qz=" << std::setw(8) << q.z()
      << "  qw=" << std::setw(8) << q.w() << "\n"
      << std::setprecision(2)
      << "  RPY (deg)  : roll=" << std::setw(7) << r2d(roll)
      << "  pitch=" << std::setw(7) << r2d(pitch)
      << "  yaw=" << std::setw(7) << r2d(yaw) << "\n";
  }

  // ─── Triangulación + tabla en terminal ───────────────────────────────────
  void triangulateAndPublish(
    const std::vector<cv::KeyPoint> & kp_prev,
    const std::vector<cv::KeyPoint> & kp_curr,
    const std::vector<cv::DMatch>   & matches,
    const Eigen::Isometry3d         & T_prev_cam,
    const Eigen::Isometry3d         & T_curr_cam,
    double                            baseline,
    const rclcpp::Time              & stamp)
  {
    cv::Mat P_prev = makeProjection(T_prev_cam, K_);
    cv::Mat P_curr = makeProjection(T_curr_cam, K_);

    std::vector<cv::Point2f> pts2d_prev, pts2d_curr;
    pts2d_prev.reserve(matches.size());
    pts2d_curr.reserve(matches.size());
    for (const auto & m : matches) {
      pts2d_prev.push_back(kp_prev[m.queryIdx].pt);
      pts2d_curr.push_back(kp_curr[m.trainIdx].pt);
    }

    // Corrección de distorsión
    std::vector<cv::Point2f> pts2d_prev_u, pts2d_curr_u;
    cv::undistortPoints(pts2d_prev, pts2d_prev_u, K_, dist_coeffs_, cv::noArray(), K_);
    cv::undistortPoints(pts2d_curr, pts2d_curr_u, K_, dist_coeffs_, cv::noArray(), K_);

    // Triangulación (salida 4×N homogénea)
    cv::Mat pts4D;
    cv::triangulatePoints(P_prev, P_curr, pts2d_prev_u, pts2d_curr_u, pts4D);

    const int N        = pts4D.cols;
    const double MAX_D = 50.0;
    int    valid = 0;
    double sum_d = 0.0, min_d = 1e9, max_d = -1e9;

    // Recoger puntos válidos
    std::vector<Eigen::Vector3d> pts3d;
    pts3d.reserve(N);

    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n"
              << "║  Puntos 3D triangulados en frame '" << odom_frame_ << "'"
              << std::string(24 - odom_frame_.size(), ' ') << "║\n"
              << "║  Baseline = " << std::fixed << std::setprecision(4)
              << baseline << " m"
              << std::string(38, ' ') << "║\n"
              << "╠════╦═══════════╦═══════════╦═══════════╦════════════╣\n"
              << "║  # ║     X (m) ║     Y (m) ║     Z (m) ║   dist (m) ║\n"
              << "╠════╬═══════════╬═══════════╬═══════════╬════════════╣\n";

    for (int i = 0; i < N; ++i) {
      double w = static_cast<double>(pts4D.at<float>(3, i));
      if (std::abs(w) < 1e-9) continue;

      double X = pts4D.at<float>(0, i) / w;
      double Y = pts4D.at<float>(1, i) / w;
      double Z = pts4D.at<float>(2, i) / w;

      // Filtro: el punto debe estar delante de la cámara previa
      Eigen::Vector3d p_cam = T_prev_cam.inverse() * Eigen::Vector3d(X, Y, Z);
      if (p_cam.z() <= 0.0) continue;

      double dist = std::sqrt(X*X + Y*Y + Z*Z);
      if (dist > MAX_D) continue;

      ++valid;
      sum_d += dist;
      min_d  = std::min(min_d, dist);
      max_d  = std::max(max_d, dist);
      pts3d.emplace_back(X, Y, Z);

      if (verbose_pts_) {
        std::cout << std::fixed << std::setprecision(3)
          << "║" << std::setw(3)  << valid << " ║"
          << std::setw(10) << X   << " ║"
          << std::setw(10) << Y   << " ║"
          << std::setw(10) << Z   << " ║"
          << std::setw(11) << dist << " ║\n";
      }
    }

    std::cout << "╠════╩═══════════╩═══════════╩═══════════╩════════════╣\n";
    if (valid > 0) {
      std::cout << std::fixed << std::setprecision(3)
        << "║  Válidos: " << std::setw(4) << valid << "/" << std::setw(4) << N
        << "  min=" << std::setw(7) << min_d
        << " m  max=" << std::setw(7) << max_d
        << " m  media=" << std::setw(7) << (sum_d / valid) << " m  ║\n";
    } else {
      std::cout << "║  Sin puntos válidos tras filtrado.                       ║\n";
    }
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    RCLCPP_INFO(this->get_logger(),
      "Triangulados %d/%d puntos | dist media=%.3f m",
      valid, N, valid > 0 ? sum_d / valid : 0.0);

    // ── Publicar PointCloud2 en el frame odom ─────────────────────────
    if (pts3d.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp    = stamp;
    cloud_msg.header.frame_id = odom_frame_;
    cloud_msg.height          = 1;                   // nube no organizada
    cloud_msg.width           = static_cast<uint32_t>(pts3d.size());
    cloud_msg.is_dense        = false;
    cloud_msg.is_bigendian    = false;

    // Definir campos: x, y, z (float32)
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pts3d.size());

    // Rellenar puntos
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud_msg, "z");
    for (const auto & p : pts3d) {
      *it_x = static_cast<float>(p.x());
      *it_y = static_cast<float>(p.y());
      *it_z = static_cast<float>(p.z());
      ++it_x; ++it_y; ++it_z;
    }

    pub_cloud_->publish(cloud_msg);
    RCLCPP_INFO(this->get_logger(),
      "PointCloud2 publicada: %zu puntos en frame '%s'",
      pts3d.size(), odom_frame_.c_str());
  }

  // ─── ¿El robot se ha movido lo suficiente? ────────────────────────────────
  bool hasMoved(const geometry_msgs::msg::Pose & p1,
                const geometry_msgs::msg::Pose & p2) const
  {
    double dx = p2.position.x - p1.position.x;
    double dy = p2.position.y - p1.position.y;
    double dz = p2.position.z - p1.position.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    tf2::Quaternion q1, q2;
    tf2::fromMsg(p1.orientation, q1);
    tf2::fromMsg(p2.orientation, q2);
    double angle = q1.angleShortestPath(q2);

    return (dist >= skip_dist_) || (angle >= skip_angle_);
  }

  // ─── Actualizar keyframe de referencia ───────────────────────────────────
  void updatePrev(const cv::Mat                  & frame,
                  const geometry_msgs::msg::Pose & pose,
                  const rclcpp::Time             & stamp,
                  const Eigen::Isometry3d        & T_cam)
  {
    frame.copyTo(prev_frame_);
    prev_pose_  = pose;
    prev_stamp_ = stamp;
    T_prev_cam_ = T_cam;  // pose de la cámara guardada para la próxima iteración
  }

  // ─── Miembros ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  pub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  cv::Ptr<cv::ORB>       orb_;
  cv::Ptr<cv::BFMatcher> matcher_;

  // Estado de imagen y odometría
  cv::Mat                   prev_frame_;
  rclcpp::Time              prev_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Pose  prev_pose_;
  geometry_msgs::msg::Pose  current_pose_;

  // Pose de la cámara en odom del keyframe anterior
  Eigen::Isometry3d T_prev_cam_ = Eigen::Isometry3d::Identity();

  // TF fijo base_link → camera (leído una vez)
  Eigen::Isometry3d T_cam_base_    = Eigen::Isometry3d::Identity();
  bool              static_tf_ready_ = false;

  // Calibración de la cámara
  cv::Mat K_;
  cv::Mat dist_coeffs_;
  bool    camera_info_received_ = false;
  bool    odom_received_        = false;

  // Parámetros
  bool        show_window_      = true;
  bool        verbose_pts_      = true;
  double      lowe_ratio_       = LOWE_RATIO;
  int         min_matches_      = MIN_MATCHES;
  double      skip_dist_        = SKIP_DIST_M;
  double      skip_angle_       = SKIP_ANGLE_RAD;
  std::string camera_frame_     = "camera_link";
  std::string base_link_frame_  = "base_link";
  std::string odom_frame_       = "odom_drone";
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrbMatcherNode>());
  rclcpp::shutdown();
  return 0;
}