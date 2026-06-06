/*
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class DataPreprocess
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;

    cv::Mat img_input_;

    DataPreprocess(Params &params)
        : cloud_input_(new pcl::PointCloud<pcl::PointXYZ>)
    {
        string bag_path = params.bag_path;
        string camera_topic = params.camera_topic;
        string lidar_topic = params.lidar_topic;
        string output_path = params.output_path;
        double rotate_lidar = params.rotate_lidar;

        std::cout << "Loading image from topic: " << camera_topic << std::endl;

        if (!loadImageFromBag(bag_path, camera_topic, img_input_))
        {
            std::cerr << "Failed to load image from bag." << std::endl;
            return;
        }

        // Try to load point cloud from ROS2 bag file
        std::cout << "Attempting to load point cloud from ROS2 bag: " << bag_path << std::endl;
        std::cout << "Looking for topic: " << lidar_topic << std::endl;

        loadPointCloudFromBag(bag_path, lidar_topic, cloud_input_);
        // pcl::io::savePLYFile(output_path + "/cloud_input.ply", *cloud_input_);
        applyLidarRotation(cloud_input_, rotate_lidar);
    }

private:
    bool loadPointCloudFromBag(
        const std::string &bag_path,
        const std::string &topic_name,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        try
        {
            // Create bag reader
            rosbag2_cpp::Reader reader;
            reader.open(bag_path);

            // Set serialization format
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

            while (reader.has_next())
            {
                auto bag_message = reader.read_next();

                // Check whether this is the target topic
                if (bag_message->topic_name != topic_name)
                {
                    continue;
                }

                // Deserialize the message
                auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&extracted_serialized_msg, ros_msg.get());

                // Convert to PCL point cloud
                pcl::PointCloud<pcl::PointXYZ>::Ptr frame(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(*ros_msg, *frame);
                *cloud += *frame;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading bag file: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    void applyLidarRotation(
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        double rotation_deg)
    {
        // Grad -> Radiant
        double theta = rotation_deg * M_PI / 180.0;

        // Transformationsmatrix
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        // Beispiel: Rotation um X-Achse
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

        // Punktwolke transformieren
        pcl::transformPointCloud(*cloud, *cloud, transform);
    }

    bool loadImageFromBag(
        const std::string &bag_path,
        const std::string &topic_name,
        cv::Mat &image)
    {
        try
        {
            rosbag2_cpp::Reader reader;
            reader.open(bag_path);

            rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

            while (reader.has_next())
            {
                auto bag_message = reader.read_next();

                if (bag_message->topic_name != topic_name)
                {
                    continue;
                }

                auto ros_msg =
                    std::make_shared<sensor_msgs::msg::Image>();

                rclcpp::SerializedMessage extracted_serialized_msg(
                    *bag_message->serialized_data);

                serialization.deserialize_message(
                    &extracted_serialized_msg,
                    ros_msg.get());

                // ROS Image -> OpenCV
                image = cv_bridge::toCvCopy(
                            ros_msg,
                            sensor_msgs::image_encodings::BGR8)
                            ->image;

                std::cout << "Successfully loaded image from bag."
                        << std::endl;

                return true;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading image from bag: "
                    << e.what() << std::endl;
        }

        return false;
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif