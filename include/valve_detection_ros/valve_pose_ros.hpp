#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "valve_detection/depth_image_processing.hpp"
#include "valve_detection/pose_estimator.hpp"
#include "valve_detection/types.hpp"

#include <memory>
#include <string>
#include <vector>
#include "vortex_msgs/msg/landmark.hpp"
#include "vortex_msgs/msg/landmark_array.hpp"
#include "vortex_msgs/msg/landmark_subtype.hpp"
#include "vortex_msgs/msg/landmark_type.hpp"

namespace valve_detection {

class ValvePoseNode : public rclcpp::Node {
   public:
    explicit ValvePoseNode(const rclcpp::NodeOptions& options);

   private:
    void depth_camera_info_cb(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    BoundingBox to_bbox(const vision_msgs::msg::BoundingBox2D& bbox) const;

    // Returns undistorted copy of bbox (center_x/y corrected for lens
    // distortion).
    BoundingBox undistort_bbox(const BoundingBox& bbox) const;

    // Non-maximum suppression: returns indices of kept detections (max 2).
    // Two boxes are duplicates when IoMin (intersection / min-area) or IoU
    // exceeds iou_duplicate_threshold_.
    std::vector<size_t> filter_duplicate_detections(
        const vision_msgs::msg::Detection2DArray& det) const;

    void publish_pose_array(const std::vector<Pose>& poses,
                            const std_msgs::msg::Header& header);

    void publish_landmarks(const std::vector<Pose>& poses,
                           const std_msgs::msg::Header& header);

    // Sync callback: depth + color + det
    void sync_cb(const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                 const sensor_msgs::msg::Image::ConstSharedPtr& color,
                 const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det);

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>;

    bool got_info_{false};
    bool got_depth_info_{false};

    // params
    bool use_color_image_{true};
    bool visualize_detections_{true};
    bool debug_visualize_{false};
    float iou_duplicate_threshold_{0.5f};
    std::string output_frame_id_{};

    int landmark_type_{1};
    int landmark_subtype_{0};

    // distortion (populated from color camera_info)
    cv::Mat cv_camera_matrix_;
    cv::Mat cv_dist_coeffs_;

    // camera data (owned by ROS node, passed to estimator and depth functions)
    ImageProperties color_props_{};
    ImageProperties depth_props_{};
    DepthColorExtrinsic depth_color_extrinsic_{};

    // estimator
    std::unique_ptr<PoseEstimator> detector_;

    // subs
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        depth_cam_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // pubs
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_stamped_pub_;
    rclcpp::Publisher<vortex_msgs::msg::LandmarkArray>::SharedPtr landmark_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colormap_pub_;

    float depth_colormap_vmin_{0.1f};
    float depth_colormap_vmax_{1125.5f};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr annulus_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        valve_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        depth_cloud_pub_;
};

}  // namespace valve_detection
