/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include "qr_detect.hpp"
#include "lidar_detect.hpp"
#include "data_preprocess.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mono_qr_pattern");

  // Load parameters
  Params params = loadParameters(node);

  // Initialize QR detection and LiDAR detection
  QRDetectPtr qrDetectPtr;
  qrDetectPtr.reset(new QRDetect(node, params));

  LidarDetectPtr lidarDetectPtr;
  lidarDetectPtr.reset(new LidarDetect(node, params));

  DataPreprocessPtr dataPreprocessPtr;
  dataPreprocessPtr.reset(new DataPreprocess(params));

  // Load image and point cloud
  cv::Mat img_input = dataPreprocessPtr->img_input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input = dataPreprocessPtr->cloud_input_;

  // Detect QR code
  PointCloud<PointXYZ>::Ptr qr_center_cloud(new PointCloud<PointXYZ>);
  qr_center_cloud->reserve(4);
  qrDetectPtr->detect_qr(img_input, qr_center_cloud);

  // Detect LiDAR data
  PointCloud<PointXYZ>::Ptr lidar_center_cloud(new PointCloud<PointXYZ>);
  lidar_center_cloud->reserve(4);
  lidarDetectPtr->detect_lidar(cloud_input, lidar_center_cloud);

  // Sort the circle centers detected by QR and LiDAR
  PointCloud<PointXYZ>::Ptr qr_centers(new PointCloud<PointXYZ>);
  PointCloud<PointXYZ>::Ptr lidar_centers(new PointCloud<PointXYZ>);
  sortPatternCenters(qr_center_cloud, qr_centers, "camera");
  sortPatternCenters(lidar_center_cloud, lidar_centers, "lidar");

  // Save intermediate results: sorted LiDAR circle centers and QR circle centers (for multi-scene calibration)
  saveTargetHoleCenters(lidar_centers, qr_centers, params);

  // Compute extrinsic parameters
  Eigen::Matrix4f transformation;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
  svd.estimateRigidTransformation(*lidar_centers, *qr_centers, transformation);

  // Transform LiDAR point cloud into the QR code coordinate frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidar_centers(new pcl::PointCloud<pcl::PointXYZ>);
  aligned_lidar_centers->reserve(lidar_centers->size());
  alignPointCloud(lidar_centers, aligned_lidar_centers, transformation);

  double rmse = computeRMSE(qr_centers, aligned_lidar_centers);
  if (rmse > 0)
  {
    RCLCPP_INFO(node->get_logger(), "[Result] RMSE: %.4f m", rmse);
  }

  RCLCPP_INFO(node->get_logger(), "[Result] Extrinsic parameters T_cam_lidar:");
  std::cout << BOLDCYAN << std::fixed << std::setprecision(6) << transformation << RESET << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  projectPointCloudToImage(cloud_input, transformation, qrDetectPtr->cameraMatrix_, qrDetectPtr->distCoeffs_, img_input, colored_cloud);

  saveCalibrationResults(params, transformation, colored_cloud, qrDetectPtr->imageCopy_);

  auto colored_cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("colored_cloud", 1);
  auto aligned_lidar_centers_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_lidar_centers", 1);

  // Main loop
  rclcpp::Rate rate(1);
  while (rclcpp::ok())
  {
    if (DEBUG)
    {
      // Publish QR detection result
      sensor_msgs::msg::PointCloud2 qr_centers_msg;
      pcl::toROSMsg(*qr_centers, qr_centers_msg);
      qr_centers_msg.header.stamp = node->now();
      qr_centers_msg.header.frame_id = "map";
      qrDetectPtr->qr_pub_->publish(qr_centers_msg);

      // Publish LiDAR detection result
      sensor_msgs::msg::PointCloud2 lidar_centers_msg;
      pcl::toROSMsg(*lidar_centers, lidar_centers_msg);
      lidar_centers_msg.header = qr_centers_msg.header;
      lidarDetectPtr->center_pub_->publish(lidar_centers_msg);

      // Publish intermediate results
      sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getFilteredCloud(), filtered_cloud_msg);
      filtered_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->filtered_pub_->publish(filtered_cloud_msg);

      sensor_msgs::msg::PointCloud2 plane_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getPlaneCloud(), plane_cloud_msg);
      plane_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->plane_pub_->publish(plane_cloud_msg);

      sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getAlignedCloud(), aligned_cloud_msg);
      aligned_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->aligned_pub_->publish(aligned_cloud_msg);

      sensor_msgs::msg::PointCloud2 edge_cloud_msg;
      pcl::toROSMsg(*lidarDetectPtr->getEdgeCloud(), edge_cloud_msg);
      edge_cloud_msg.header = qr_centers_msg.header;
      lidarDetectPtr->edge_pub_->publish(edge_cloud_msg);

      sensor_msgs::msg::PointCloud2 lidar_centers_z0_msg;
      pcl::toROSMsg(*lidarDetectPtr->getCenterZ0Cloud(), lidar_centers_z0_msg);
      lidar_centers_z0_msg.header = qr_centers_msg.header;
      lidarDetectPtr->center_z0_pub_->publish(lidar_centers_z0_msg);

      // Publish LiDAR point cloud after extrinsic transformation
      sensor_msgs::msg::PointCloud2 aligned_lidar_centers_msg;
      pcl::toROSMsg(*aligned_lidar_centers, aligned_lidar_centers_msg);
      aligned_lidar_centers_msg.header = qr_centers_msg.header;
      aligned_lidar_centers_pub->publish(aligned_lidar_centers_msg);

      // Publish colored point cloud
      sensor_msgs::msg::PointCloud2 colored_cloud_msg;
      pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
      colored_cloud_msg.header = qr_centers_msg.header;
      colored_cloud_pub->publish(colored_cloud_msg);

      // cv::imshow("result", qrDetectPtr->imageCopy_);
    }
    // cv::waitKey(1);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}