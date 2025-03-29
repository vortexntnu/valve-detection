#ifndef DETECTION_IMAGE_PROCESSOR_HPP
#define DETECTION_IMAGE_PROCESSOR_HPP

#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <random>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class ValveDetectionNode : public rclcpp::Node {
   public:
    explicit ValveDetectionNode(const rclcpp::NodeOptions& options);
    ~ValveDetectionNode() {};

   private:
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray>
        detections_sub_;

    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>
        MySyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        camera_info_subscription_yolo_color_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
        detections_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
        image_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr plane_normal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        line_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        line_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        near_plane_cloud_pub_;

    // Stored messages
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
    bool camera_info_received_ = false;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_yolo_color_;
    bool camera_info_received_yolo_color_ = false;
    // vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
    // sensor_msgs::msg::Image::SharedPtr latest_image_;

    // Mutex for thread safety
    std::mutex mutex_;

    // Used for line fitting direction filtering
    static Eigen::Vector3f filter_direction_;

    // Scalars for image resizing
    double height_scalar_;
    double width_scalar_;

    void synchronized_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);
    // Callback functions
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
    void camera_info_callback_yolo_colored(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    // Utility functions
    void compute_height_width_scalars();
    // void process_and_publish_image(const
    // sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    //                             const sensor_msgs::msg::Image::ConstSharedPtr
    //                             & color_image, const
    //                             vision_msgs::msg::Detection2DArray::ConstSharedPtr
    //                             & detections);
    void process_and_publish_image(
        const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    void project_pixel_to_3d(int u, int v, float depth, pcl::PointXYZ& point);
    void project_3d_to_pixel(float x, float y, float z, int& u, int& v);
};

#endif  // DETECTION_IMAGE_PROCESSOR_HPP
