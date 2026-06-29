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
#include <image_transport/subscriber_filter.hpp>
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
    // Image transport: "compressed" pulls only the JPEG stream over the network
    // instead of the raw (~20x larger) frames. Set to "raw" for a local setup.
    node_->declare_parameter("camera_transport", std::string("compressed"));
    // Stability gate: a scene is only captured once the board has held still for
    // a few consecutive frames. A moving board makes the camera frame and the
    // (rolling) LiDAR sweep disagree about its pose, which is the dominant source
    // of live calibration error. Thresholds are the max board motion between two
    // consecutive processed frames that still counts as "still".
    node_->declare_parameter("require_still", true);
    node_->declare_parameter("still_translation_thresh", 0.006);  // m
    node_->declare_parameter("still_rotation_thresh", 0.6);       // deg
    node_->declare_parameter("still_frames", 3);                  // consecutive still frames

    params_ = loadParameters(node_);

    node_->get_parameter_or("target_scenes",        target_scenes_,    5);
    node_->get_parameter_or("min_translation_diff", min_trans_diff_,   0.05);
    double rot_deg = 5.0;
    node_->get_parameter_or("min_rotation_diff",    rot_deg,           5.0);
    min_rot_diff_rad_ = rot_deg * M_PI / 180.0;

    node_->get_parameter_or("require_still",            require_still_,      true);
    node_->get_parameter_or("still_translation_thresh", still_trans_thresh_, 0.006);
    double still_rot_deg = 0.6;
    node_->get_parameter_or("still_rotation_thresh",    still_rot_deg,       0.6);
    still_rot_thresh_rad_ = still_rot_deg * M_PI / 180.0;
    node_->get_parameter_or("still_frames",             still_frames_,       3);

    qr_detect_    = std::make_shared<QRDetect>(node_, params_);
    lidar_detect_ = std::make_shared<LidarDetect>(node_, params_);

    // Live mode runs the detection pipeline on every synchronized frame. The
    // per-frame debug PLY snapshots are pure disk I/O on the callback thread and
    // would block it for hundreds of ms each frame, so only write them when debug
    // is explicitly enabled. The final result clouds are still saved by
    // triggerCalibration() regardless.
    lidar_detect_->setWriteDebugClouds(params_.debug);

    colored_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
    detection_img_pub_ =
      node_->create_publisher<sensor_msgs::msg::Image>("detection_image", 1);

    // Sensor streams arrive over DDS from a remote machine (e.g. a Jetson).
    //
    // Camera: subscribe via image_transport with the "compressed" transport.
    // Subscribing to raw image_raw forces the publisher to push the full
    // ~1.1 Gbit/s uncompressed stream onto the network, which saturates the link
    // and collapses the rate of every topic (including the compressed one). The
    // compressed transport pulls only the ~50 Mbit/s JPEG stream and decodes it
    // locally, so the wire never carries the raw frames. RELIABLE with a small
    // queue: a JPEG frame still spans a few RTPS fragments, so reliability avoids
    // dropped frames, and the callback is cheap enough not to back-pressure.
    std::string transport;
    node_->get_parameter_or("camera_transport", transport, std::string("compressed"));
    rmw_qos_profile_t image_qos = rmw_qos_profile_default;  // RELIABLE, keep_last
    image_qos.depth = 2;                                    // small queue: no stale backlog
    img_sub_ = std::make_shared<image_transport::SubscriberFilter>();
    img_sub_->subscribe(node_.get(), params_.camera_topic, transport, image_qos);

    // LiDAR: use SENSOR_DATA (BEST_EFFORT). Point clouds tolerate dropped frames
    // and best-effort is the standard sensor profile; it also stays compatible
    // whether the driver publishes reliable or best-effort.
    cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                  node_, params_.lidar_topic, rmw_qos_profile_sensor_data);

    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Camera image_transport: '%s'.", transport.c_str());

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

    // Throttle the heavy QR + LiDAR detection. Calibration only needs occasional
    // frames (a human repositioning the board), but running the full pipeline on
    // every synchronized frame keeps the executor thread busy and back-pressures
    // the reliable image subscription. Returning early on most frames keeps the
    // callback cheap so the sensor topics stay at full rate.
    rclcpp::Time now = node_->now();
    if (last_process_time_.nanoseconds() != 0 &&
        (now - last_process_time_).seconds() < min_process_interval_s_)
      return;
    last_process_time_ = now;

    // Convert image to MONO8. ArUco detection works on grayscale anyway, so we
    // ask cv_bridge for the most direct path to gray. For a Bayer source this
    // demosaics straight to luminance and avoids reconstructing colour (and the
    // JPEG-on-Bayer colour/zipper artifacts that would hurt corner accuracy).
    // detect_qr builds its own 3-channel image for the coloured preview overlay.
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(img_msg,
              sensor_msgs::image_encodings::MONO8)->image;
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

    // Publish the (locally decoded) annotated image every processed frame so the
    // operator always has a live preview in RViz — even when no target is found.
    // detect_qr always copies the input into imageCopy_ and overlays detections
    // on top, so this carries both the camera view and the detection quality.
    // This is a LOCAL publication; RViz should view this, not the remote raw
    // topic, otherwise it pulls the heavy raw stream over the network again.
    auto det_msg = cv_bridge::CvImage(
        img_msg->header, "bgr8", qr_detect_->imageCopy_).toImageMsg();
    detection_img_pub_->publish(*det_msg);

    if (!qr_detect_->last_detection_valid_ || qr_cloud->size() != 4)
      return;  // detect_qr already emits a WARN; avoid flooding the log

    // Stability gate: only proceed once the board has been (nearly) motionless
    // for several consecutive frames. The board pose comes from the camera pose
    // estimate (tvec/rvec); comparing it frame-to-frame tells us whether it is
    // moving. Capturing while it moves makes the camera and the rolling LiDAR
    // sweep disagree about the pose — the main live-calibration error source.
    if (require_still_ && !isStill(qr_detect_->last_tvec_, qr_detect_->last_rvec_))
      return;

    // LiDAR detection — clear accumulating clouds before reuse
    lidar_detect_->resetIntermediateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_detect_->detect_lidar(cloud, lidar_cloud);

    if (lidar_cloud->size() != 4) {
      if (params_.debug)
        RCLCPP_WARN(node_->get_logger(),
          "[LiveCalib] LiDAR: found %zu circles (need 4). Skipping frame.",
          lidar_cloud->size());
      return;
    }

    // Scene diversity check
    if (!isDiverse(qr_detect_->last_tvec_, qr_detect_->last_rvec_)) {
      if (params_.debug)
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

    // Running joint RMSE over all scenes captured so far, for live feedback.
    // With a single scene the fit is near-degenerate, so only report it once
    // there are at least two scenes to disagree.
    if (scenes_.size() >= 2) {
      double rms = currentRMSE();
      RCLCPP_INFO(node_->get_logger(),
        "[LiveCalib] Scene %zu / %d captured.  Running RMSE: %.4f m",
        scenes_.size(), target_scenes_, rms);
    } else {
      RCLCPP_INFO(node_->get_logger(),
        "[LiveCalib] Scene %zu / %d captured.",
        scenes_.size(), target_scenes_);
    }

    if (static_cast<int>(scenes_.size()) >= target_scenes_)
      triggerCalibration(cloud, img, img_msg->header);
  }

  // Joint RMSE over all scenes captured so far. Returns -1 if not solvable.
  double currentRMSE() const
  {
    std::vector<Eigen::Vector3d> L, C;
    for (const auto& sc : scenes_)
      for (int i = 0; i < 4; ++i) {
        L.push_back(sc.lidar_pts[i]);
        C.push_back(sc.qr_pts[i]);
      }
    auto res = solveRigid(L, C);
    return res.ok ? res.rms : -1.0;
  }

  // ── Board stability check ─────────────────────────────────────────────────

  // Returns true once the board pose has stayed within the motion thresholds for
  // `still_frames_` consecutive processed frames. Updates the rolling state.
  bool isStill(const cv::Vec3d& tvec, const cv::Vec3d& rvec)
  {
    bool still_now = true;
    if (have_prev_pose_) {
      double dt = cv::norm(tvec - prev_tvec_);
      double dr = cv::norm(rvec - prev_rvec_);
      still_now = (dt < still_trans_thresh_ && dr < still_rot_thresh_rad_);
    } else {
      still_now = false;  // need at least one previous frame to compare
    }
    prev_tvec_ = tvec;
    prev_rvec_ = rvec;
    have_prev_pose_ = true;

    if (still_now) {
      ++stable_count_;
    } else {
      if (stable_count_ >= still_frames_ && params_.debug)
        RCLCPP_INFO(node_->get_logger(), "[LiveCalib] Board moving — hold still to capture.");
      stable_count_ = 0;
    }
    return stable_count_ >= still_frames_;
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

    // Route the result through the ROS logger (not std::cout) so it stays in
    // order with the other [LiveCalib] messages under `ros2 launch`.
    std::ostringstream mat;
    mat << std::fixed << std::setprecision(6) << T;
    RCLCPP_INFO(node_->get_logger(),
      "[LiveCalib] Joint calibration complete. RMSE: %.4f m\nT_cam_lidar:\n%s",
      res.rms, mat.str().c_str());

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
    RCLCPP_INFO(node_->get_logger(), "[LiveCalib] Results saved to %s", path.c_str());
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

  // Stability gate state
  bool      require_still_         = true;
  double    still_trans_thresh_    = 0.006;
  double    still_rot_thresh_rad_  = 0.6 * M_PI / 180.0;
  int       still_frames_          = 3;
  bool      have_prev_pose_        = false;
  int       stable_count_          = 0;
  cv::Vec3d prev_tvec_;
  cv::Vec3d prev_rvec_;

  QRDetectPtr    qr_detect_;
  LidarDetectPtr lidar_detect_;

  std::vector<Scene> scenes_;
  bool calibrated_ = false;

  // Detection throttle: process at most ~3 Hz regardless of incoming rate.
  rclcpp::Time last_process_time_{0, 0, RCL_ROS_TIME};
  double       min_process_interval_s_ = 0.33;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr       detection_img_pub_;

  std::shared_ptr<image_transport::SubscriberFilter>
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
