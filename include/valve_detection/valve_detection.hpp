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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr line_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr line_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr near_plane_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr canny_debug_image_pub_;

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

    int line_detection_area_;

    /**
     * @brief Callback function for synchronized depth image, color image, and 2D detections.
     *
     * This function is triggered when synchronized messages for a depth image, a color image, and a 2D detection array
     */
    void synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr & depth_image, 
                           const sensor_msgs::msg::Image::ConstSharedPtr & color_image, 
                           const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections);
    // Callback functions
    void camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);
    void camera_info_callback_yolo_colored(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    // Utility functions
    /**
     * @brief Computes the height and width scalars based on the camera information.
     *
     * This function calculates the scaling factors (`height_scalar_` and `width_scalar_`) 
     * by comparing the dimensions of two camera frames:
     * - The YOLO color camera frame (`camera_info_yolo_color_`).
     * - The reference camera frame (`camera_info_`).
     *
    */
    void compute_height_width_scalars();

    /**
     * @brief Main logic executed when two images are synchronized.
     *
     * This function processes synchronized depth and color images along with 2D detections to calculate the angle of the valve handle
     * and its orientation in 3D space. It uses RANSAC line detection to determine the handle's angle and RANSAC plane detection
     * to find the normal vector of the plane in which the valve handle lies.
     *
     * The function performs the following steps:
     * 1. Converts the depth and color images from ROS messages to OpenCV matrices.
     * 2. Selects the most centered detection from the 2D detection array.
     * 3. Projects the detected bounding box into 3D space using the depth image and camera intrinsics.
     * 4. Uses RANSAC plane detection to find the plane normal vector and filters points near the plane.
     * 5. Detects the valve handle's orientation using Hough line detection on the color image.
     * 6. Computes the 3D pose of the valve handle and publishes it as a `geometry_msgs::msg::PoseStamped` message.
     *
     * @param[in] depth_image A shared pointer to the depth image message.
     * @param[in] color_image A shared pointer to the color image message.
     * @param[in] detections A shared pointer to the 2D detection array message.
     *
     * @pre The `camera_info_` and `camera_info_yolo_color_` member variables must be initialized with valid camera intrinsic parameters.
     * @pre The input images and detections must be synchronized (i.e., they correspond to the same timestamp or frame).
     * @post If successful, the function publishes the following:
     *       - A point cloud of the detected valve handle region.
     *       - The normal vector of the detected plane.
     *       - The 3D pose of the valve handle.
     * @post If unsuccessful, the function logs warnings or errors and returns early.
     *
     * @note The function assumes that the input images and detections are synchronized.
     * @note The function uses RANSAC for robust plane and line detection, which is suitable for noisy sensor data.
     * @note The function handles edge cases such as invalid depth values, empty detections, and failed RANSAC fits.
     */
    void process_and_publish_image(const sensor_msgs::msg::Image::ConstSharedPtr & depth_image, 
                                const sensor_msgs::msg::Image::ConstSharedPtr & color_image, 
                                const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections);

    /**
     * @brief Projects a 2D pixel coordinate and depth value into a 3D point in the camera frame.
     *
     * This function converts a pixel coordinate `(u, v)` and its corresponding depth value into a 3D point
    */
    void project_pixel_to_3d(int u, int v, float depth, pcl::PointXYZ &point);

    /**
     * @brief Projects a 3D point in the camera frame to 2D pixel coordinates.
     *
     * This function converts a 3D point `(x, y, z)` in the camera frame into 2D pixel coordinates `(u, v)`
    */
    void project_3d_to_pixel(float x, float y, float z, int &u, int &v);
};

#endif  // DETECTION_IMAGE_PROCESSOR_HPP
