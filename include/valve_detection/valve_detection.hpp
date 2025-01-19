#ifndef DETECTION_IMAGE_PROCESSOR_HPP
#define DETECTION_IMAGE_PROCESSOR_HPP

#include <memory>
#include <vector>
#include <iostream>
#include <cmath>
#include <random>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <Eigen/Dense> 


#include <opencv2/opencv.hpp>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>

class DetectionImageProcessor : public rclcpp::Node
{
public:
    DetectionImageProcessor();

private:
    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_yolo_color_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr plane_normal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr line_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr line_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_plane_cloud_pub_;

    // Stored messages
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    bool camera_info_received_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_yolo_color_;
    bool camera_info_received_yolo_color_;
    vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;

    // Mutex for thread safety
    std::mutex mutex_;

    // Scalars for image resizing
    double height_scalar_;
    double width_scalar_;

    // Callback functions
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
    void camera_info_callback_yolo_colored(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
    void detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr detections_msg);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);

    // Utility functions
    void compute_height_width_scalars();
    void process_and_publish_image();
    void project_pixel_to_3d(int u, int v, float depth, pcl::PointXYZ &point);
    void project_3d_to_pixel(float x, float y, float z, int &u, int &v);
};

#endif // DETECTION_IMAGE_PROCESSOR_HPP
