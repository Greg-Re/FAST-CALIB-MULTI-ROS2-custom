/*
Live calibration node for FAST-CALIB.

Subscribes to camera and LiDAR topics, automatically collects calibration
scenes, and runs joint multi-scene calibration once enough scenes have been
captured.  Replaces the manual bag-record → calib.launch → multi_calib.launch
workflow with a single continuously running node.

Usage:
  ros2 launch fast_calib live_calib.launch.py
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <cmath>

#include "qr_detect.hpp"
#include "lidar_detect.hpp"

// ── Joint rigid-body solver (mirrors multi_scene.cpp) ────────────────────────

struct RigidResult
{
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double rms = 0.0;
  bool ok = false;
};

static RigidResult solveRigid(const std::vector<Eigen::Vector3d>& L,
                               const std::vector<Eigen::Vector3d>& C)
{
  RigidResult out;
  const size_t N = L.size();
  if (N < 3 || C.size() != N) return out;

  const double wsum = static_cast<double>(N);
  Eigen::Vector3d muL = Eigen::Vector3d::Zero();
  Eigen::Vector3d muC = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < N; ++i) { muL += L[i]; muC += C[i]; }
  muL /= wsum; muC /= wsum;

  Eigen::Matrix3d Sigma = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < N; ++i)
    Sigma += (L[i] - muL) * (C[i] - muC).transpose();

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(Sigma,
    Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0) {
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = -1;
    R = svd.matrixV() * D * svd.matrixU().transpose();
  }
  Eigen::Vector3d t = muC - R * muL;

  double rss = 0.0;
  for (size_t i = 0; i < N; ++i) {
    Eigen::Vector3d r = (R * L[i] + t) - C[i];
    rss += r.squaredNorm();
  }
  out.R = R; out.t = t; out.rms = std::sqrt(rss / wsum); out.ok = true;
  return out;
}

// ── Live calibration node ─────────────────────────────────────────────────────

class LiveCalibNode
{
public:
  LiveCalibNode()
  {
    node_ = std::make_shared<rclcpp::Node>("fast_calib");

    // Declare live-mode params before loadParameters so they are in the
    // parameter server when the YAML file is applied.
    node_->declare_parameter("target_scenes", 5);
    node_->declare_parameter("min_translation_diff", 0.05);
    node_->declare_parameter("min_rotation_diff", 5.0);

    params_ = loadParameters(node_);

    node_->get_parameter_or("target_scenes",        target_scenes_,    5);
    node_->get_parameter_or("min_translation_diff", min_trans_diff_,   0.05);
    double rot_deg = 5.0;
    node_->get_parameter_or("min_rotation_diff",    rot_deg,           5.0);
    min_rot_diff_rad_ = rot_deg * M_PI / 180.0;

    qr_detect_    = std::make_shared<QRDetect>(node_, params_);
    lidar_detect_ = std::make_shared<LidarDetect>(node_, params_);

    colored_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    detection_img_pub_ =
      node_->create_publisher<sensor_msgs::msg::Image>("detection_image", 1);

    img_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
                  node_, params_.camera_topic);
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                  node_, params_.lidar_topic);

    sync_ = std::make_shared<Sync>(SyncPolicy(20), *img_sub_, *cloud_sub_);
    sync_->registerCallback(
      std::bind(&LiveCalibNode::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Started. Target: %d scenes | "
      "min_translation=%.3f m | min_rotation=%.1f deg.",
      target_scenes_, min_trans_diff_, rot_deg);
    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Listening on camera='%s', lidar='%s'.",
      params_.camera_topic.c_str(), params_.lidar_topic.c_str());
  }

  rclcpp::Node::SharedPtr getNode() { return node_; }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::PointCloud2>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  struct Scene
  {
    std::vector<Eigen::Vector3d> lidar_pts;  // 4 sorted LiDAR circle centres
    std::vector<Eigen::Vector3d> qr_pts;     // 4 sorted camera circle centres
    cv::Vec3d tvec;                          // board→camera translation (diversity)
    cv::Vec3d rvec;                          // board→camera rotation / Rodrigues
  };

  // ── Synchronised callback ─────────────────────────────────────────────────

  void syncCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr&       img_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg)
  {
    if (calibrated_) {
      publishStoredCloud(img_msg->header);
      return;
    }

    // Convert image
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(img_msg,
              sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN(node_->get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // Convert + optionally rotate point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (params_.rotate_lidar != 0.0) {
      float theta = static_cast<float>(params_.rotate_lidar * M_PI / 180.0);
      Eigen::Affine3f tf = Eigen::Affine3f::Identity();
      tf.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
      pcl::transformPointCloud(*cloud, *cloud, tf);
    }

    // QR detection
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    qr_detect_->detect_qr(img, qr_cloud);

    if (!qr_detect_->last_detection_valid_ || qr_cloud->size() != 4)
      return;  // detect_qr already emits a WARN; avoid flooding the log

    // Publish annotated image so operator can see detection quality
    auto det_msg = cv_bridge::CvImage(
        img_msg->header, "bgr8", qr_detect_->imageCopy_).toImageMsg();
    detection_img_pub_->publish(*det_msg);

    // LiDAR detection — clear accumulating clouds before reuse
    lidar_detect_->resetIntermediateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_detect_->detect_lidar(cloud, lidar_cloud);

    if (lidar_cloud->size() != 4) {
      RCLCPP_WARN(node_->get_logger(),
        "[LiveCalib] LiDAR: found %zu circles (need 4). Skipping frame.",
        lidar_cloud->size());
      return;
    }

    // Scene diversity check
    if (!isDiverse(qr_detect_->last_tvec_, qr_detect_->last_rvec_)) {
      RCLCPP_INFO(node_->get_logger(),
        "[LiveCalib] Scene too similar to an existing one. "
        "Move the calibration target to a new position.");
      return;
    }

    // Sort centres for consistent correspondence
    pcl::PointCloud<pcl::PointXYZ>::Ptr qr_sorted(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_sorted(new pcl::PointCloud<pcl::PointXYZ>);
    sortPatternCenters(qr_cloud,    qr_sorted,    "camera");
    sortPatternCenters(lidar_cloud, lidar_sorted, "lidar");

    Scene sc;
    sc.tvec = qr_detect_->last_tvec_;
    sc.rvec = qr_detect_->last_rvec_;
    for (int i = 0; i < 4; ++i) {
      sc.lidar_pts.emplace_back(
        lidar_sorted->at(i).x, lidar_sorted->at(i).y, lidar_sorted->at(i).z);
      sc.qr_pts.emplace_back(
        qr_sorted->at(i).x, qr_sorted->at(i).y, qr_sorted->at(i).z);
    }
    scenes_.push_back(sc);

    // Append to circle_center_record.txt (keeps compatibility with multi_calib)
    saveTargetHoleCenters(lidar_sorted, qr_sorted, params_);

    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Scene %zu / %d captured.",
      scenes_.size(), target_scenes_);

    if (static_cast<int>(scenes_.size()) >= target_scenes_)
      triggerCalibration(cloud, img, img_msg->header);
  }

  // ── Scene diversity check ─────────────────────────────────────────────────

  bool isDiverse(const cv::Vec3d& tvec, const cv::Vec3d& rvec) const
  {
    for (const auto& sc : scenes_) {
      // Euclidean distance in translation space
      double dt = cv::norm(tvec - sc.tvec);
      // Rodrigues vector difference (approximates rotation angle for small diffs)
      double dr = cv::norm(rvec - sc.rvec);
      if (dt < min_trans_diff_ && dr < min_rot_diff_rad_)
        return false;
    }
    return true;
  }

  // ── Joint calibration ─────────────────────────────────────────────────────

  void triggerCalibration(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& last_cloud,
    const cv::Mat&                              last_img,
    const std_msgs::msg::Header&                hdr)
  {
    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Running joint calibration over %zu scenes…", scenes_.size());

    std::vector<Eigen::Vector3d> L, C;
    for (const auto& sc : scenes_)
      for (int i = 0; i < 4; ++i) {
        L.push_back(sc.lidar_pts[i]);
        C.push_back(sc.qr_pts[i]);
      }

    auto res = solveRigid(L, C);
    if (!res.ok) {
      RCLCPP_ERROR(node_->get_logger(), "[LiveCalib] Joint solver failed.");
      return;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = res.R.cast<float>();
    T.block<3, 1>(0, 3) = res.t.cast<float>();

    std::cout << BOLDYELLOW << "\n[LiveCalib] Joint calibration complete." << RESET
              << std::endl;
    std::cout << BOLDYELLOW << "[LiveCalib] RMSE: " << BOLDRED
              << std::fixed << std::setprecision(4) << res.rms << " m" << RESET
              << std::endl;
    std::cout << BOLDYELLOW << "[LiveCalib] T_cam_lidar:\n" << BOLDCYAN
              << std::fixed << std::setprecision(6) << T << RESET << std::endl;

    saveMultiResult(res, static_cast<int>(scenes_.size()));

    // Colour the last received cloud and save calib_result.txt + PLY/PCD
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    projectPointCloudToImage(
      last_cloud, T,
      qr_detect_->cameraMatrix_, qr_detect_->distCoeffs_,
      last_img, colored);
    saveCalibrationResults(params_, T, colored, last_img);

    colored_cloud_ = colored;
    calibrated_    = true;

    publishStoredCloud(hdr);
  }

  // ── Helpers ───────────────────────────────────────────────────────────────

  void saveMultiResult(const RigidResult& res, int n_scenes)
  {
    std::string path = params_.output_path;
    if (path.back() != '/') path += '/';
    path += "multi_calib_result.txt";

    std::ofstream f(path);
    if (!f.is_open()) {
      RCLCPP_WARN(node_->get_logger(),
        "[LiveCalib] Cannot write %s", path.c_str());
      return;
    }
    f << "# FAST-LIVO2 calibration format (live multi-scene)\n"
      << std::fixed << std::setprecision(6);
    f << "Rcl: [ "
      << std::setw(9) << res.R(0,0) << ", "
      << std::setw(9) << res.R(0,1) << ", "
      << std::setw(9) << res.R(0,2) << ",\n"
      << "      "
      << std::setw(9) << res.R(1,0) << ", "
      << std::setw(9) << res.R(1,1) << ", "
      << std::setw(9) << res.R(1,2) << ",\n"
      << "      "
      << std::setw(9) << res.R(2,0) << ", "
      << std::setw(9) << res.R(2,1) << ", "
      << std::setw(9) << res.R(2,2) << "]\n";
    f << "Pcl: [ "
      << std::setw(9) << res.t(0) << ", "
      << std::setw(9) << res.t(1) << ", "
      << std::setw(9) << res.t(2) << "]\n";
    f << "RMSE: "        << res.rms   << " m\n";
    f << "Scenes_used: " << n_scenes  << "\n";
    f.close();
    std::cout << BOLDYELLOW << "[LiveCalib] Results saved to "
              << BOLDWHITE  << path << RESET << std::endl;
  }

  void publishStoredCloud(const std_msgs::msg::Header& hdr)
  {
    if (!colored_cloud_ || colored_cloud_->empty()) return;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*colored_cloud_, msg);
    msg.header = hdr;
    colored_cloud_pub_->publish(msg);
  }

  // ── Members ───────────────────────────────────────────────────────────────

  rclcpp::Node::SharedPtr node_;
  Params  params_;
  int     target_scenes_    = 5;
  double  min_trans_diff_   = 0.05;
  double  min_rot_diff_rad_ = 5.0 * M_PI / 180.0;

  QRDetectPtr    qr_detect_;
  LidarDetectPtr lidar_detect_;

  std::vector<Scene> scenes_;
  bool calibrated_ = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       detection_img_pub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>>
      img_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      cloud_sub_;
  std::shared_ptr<Sync> sync_;
};

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto calib = std::make_shared<LiveCalibNode>();
  rclcpp::spin(calib->getNode());
  rclcpp::shutdown();
  return 0;
}
