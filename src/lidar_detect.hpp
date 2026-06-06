/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIDAR_DETECT_HPP
#define LIDAR_DETECT_HPP
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include "common_lib.h"

class LidarDetect
{
private:
    double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    double circle_radius_;
    rclcpp::Node::SharedPtr node_;
    std::string output_path_;

    // Point clouds for storing intermediate results
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_z0_cloud_;

public:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_z0_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr center_pub_;

    LidarDetect(rclcpp::Node::SharedPtr node, Params &params)
        : node_(node),
          filtered_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          plane_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          aligned_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          edge_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          center_z0_cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        x_min_ = params.x_min;
        x_max_ = params.x_max;
        y_min_ = params.y_min;
        y_max_ = params.y_max;
        z_min_ = params.z_min;
        z_max_ = params.z_max;
        circle_radius_ = params.circle_radius;
        output_path_ = params.output_path + "/";

        filtered_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 1);
        plane_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("plane_cloud", 1);
        aligned_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 1);
        edge_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("edge_cloud", 1);
        center_z0_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("center_z0_cloud", 10);
        center_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("center_cloud", 10);
    }

    void detect_lidar(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud)
    {
        // 1. Pass-through filtering along X / Y / Z axes
        filtered_cloud_->reserve(cloud->size());

        pcl::PassThrough<pcl::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_min_, x_max_); // Set X-axis range
        pass_x.filter(*filtered_cloud_);

        pcl::PassThrough<pcl::PointXYZ> pass_y;
        pass_y.setInputCloud(filtered_cloud_);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_min_, y_max_); // Set Y-axis range
        pass_y.filter(*filtered_cloud_);

        pcl::PassThrough<pcl::PointXYZ> pass_z;
        pass_z.setInputCloud(filtered_cloud_);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_min_, z_max_); // Set Z-axis range
        pass_z.filter(*filtered_cloud_);

        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud_);
        voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);
        voxel_filter.filter(*filtered_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Filtered cloud size: %ld", filtered_cloud_->size());
        save2PLY(filtered_cloud_, output_path_ + "filtered_cloud.ply");
        // 2. Plane segmentation
        plane_cloud_->reserve(filtered_cloud_->size());

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
        plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
        plane_segmentation.setMethodType(pcl::SAC_RANSAC);
        plane_segmentation.setDistanceThreshold(0.01); // Plane segmentation distance threshold
        plane_segmentation.setInputCloud(filtered_cloud_);
        plane_segmentation.segment(*plane_inliers, *plane_coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered_cloud_);
        extract.setIndices(plane_inliers);
        extract.filter(*plane_cloud_);
        RCLCPP_INFO(node_->get_logger(), "Plane cloud size: %ld", plane_cloud_->size());
        save2PLY(plane_cloud_, output_path_ + "plane_cloud.ply");
        // 3. Align the plane point cloud
        aligned_cloud_->reserve(plane_cloud_->size());

        Eigen::Vector3d normal(plane_coefficients->values[0],
                               plane_coefficients->values[1],
                               plane_coefficients->values[2]);
        normal.normalize();
        Eigen::Vector3d z_axis(0, 0, 1);

        Eigen::Vector3d axis = normal.cross(z_axis);
        double angle = acos(normal.dot(z_axis));

        Eigen::AngleAxisd rotation(angle, axis);
        Eigen::Matrix3d R = rotation.toRotationMatrix();

        // Apply the rotation matrix to align the plane onto the Z=0 plane
        float average_z = 0.0;
        int cnt = 0;
        for (const auto &pt : *plane_cloud_)
        {
            Eigen::Vector3d point(pt.x, pt.y, pt.z);
            Eigen::Vector3d aligned_point = R * point;
            aligned_cloud_->push_back(pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
            average_z += aligned_point.z();
            cnt++;
        }
        average_z /= cnt;
        save2PLY(aligned_cloud_, output_path_ + "aligned_cloud.ply");
        // 4. Extract edge points
        edge_cloud_->reserve(aligned_cloud_->size());

        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        normal_estimator.setInputCloud(aligned_cloud_);
        normal_estimator.setRadiusSearch(0.03); // Search radius for normal estimation
        normal_estimator.compute(*normals);

        pcl::PointCloud<pcl::Boundary> boundaries;
        pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_estimator;
        boundary_estimator.setInputCloud(aligned_cloud_);
        boundary_estimator.setInputNormals(normals);
        boundary_estimator.setRadiusSearch(0.04);       // Search radius for boundary detection
        boundary_estimator.setAngleThreshold(M_PI / 4); // Angle threshold
        boundary_estimator.compute(boundaries);

        for (size_t i = 0; i < aligned_cloud_->size(); ++i)
        {
            if (boundaries.points[i].boundary_point > 0)
            {
                edge_cloud_->push_back(aligned_cloud_->points[i]);
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Extracted %ld edge points.", edge_cloud_->size());
        save2PLY(edge_cloud_, output_path_ + "edge_cloud.ply");
        // 5. Cluster the edge points
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(edge_cloud_);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.02); // Cluster distance threshold
        ec.setMinClusterSize(50);     // Minimum cluster size
        ec.setMaxClusterSize(1000);   // Maximum cluster size
        ec.setSearchMethod(tree);
        ec.setInputCloud(edge_cloud_);
        ec.extract(cluster_indices);

        RCLCPP_INFO(node_->get_logger(), "Number of edge clusters: %ld", cluster_indices.size());

        // 6. Fit a circle to each cluster
        center_z0_cloud_->reserve(4);
        Eigen::Matrix3d R_inv = R.inverse();

        // Fit a circle to each cluster
        for (size_t i = 0; i < cluster_indices.size(); ++i)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto &idx : cluster_indices[i].indices)
            {
                cluster->push_back(edge_cloud_->points[idx]);
            }

            // Circle fitting
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CIRCLE2D);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01); // Distance threshold
            seg.setMaxIterations(10000);     // Maximum number of iterations
            seg.setInputCloud(cluster);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() > 0)
            {
                // Compute fitting error
                double error = 0.0;
                for (const auto &idx : inliers->indices)
                {
                    double dx = cluster->points[idx].x - coefficients->values[0];
                    double dy = cluster->points[idx].y - coefficients->values[1];
                    double distance = sqrt(dx * dx + dy * dy) - circle_radius_; // Distance error
                    error += abs(distance);
                }
                error /= inliers->indices.size();

                // If fitting error is small enough, treat it as a circular hole
                std::cout << "inliers->indices.size() " << inliers->indices.size() << " " << error << std::endl;
                if (error < 0.02)
                {
                    // Add the recovered circle center to the point cloud
                    pcl::PointXYZ center_point;
                    center_point.x = coefficients->values[0];
                    center_point.y = coefficients->values[1];
                    center_point.z = 0.0;
                    center_z0_cloud_->push_back(center_point);

                    // Transform the circle center back to the original coordinate frame
                    Eigen::Vector3d aligned_point(center_point.x, center_point.y, center_point.z + average_z);
                    Eigen::Vector3d original_point = R_inv * aligned_point;

                    pcl::PointXYZ center_point_origin;
                    center_point_origin.x = original_point.x();
                    center_point_origin.y = original_point.y();
                    center_point_origin.z = original_point.z();
                    center_cloud->points.push_back(center_point_origin);
                }
            }
        }
    }

    // Accessors for intermediate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const { return filtered_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPlaneCloud() const { return plane_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getEdgeCloud() const { return edge_cloud_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCenterZ0Cloud() const { return center_z0_cloud_; }
};

typedef std::shared_ptr<LidarDetect> LidarDetectPtr;

#endif
