#pragma once

#include "valve_detection/depth_image_processing.hpp"
#include "valve_detection/types.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace valve_detection {

class PoseEstimator {
   public:
    PoseEstimator(int yolo_img_width,
                  int yolo_img_height,
                  float annulus_radius_ratio,
                  float plane_ransac_threshold,
                  int plane_ransac_max_iterations,
                  float valve_handle_offset);

    void set_color_image_properties(const ImageProperties& props);
    void set_depth_image_properties(const ImageProperties& props);
    void set_depth_color_extrinsic(const DepthColorExtrinsic& extr);

    void calculate_letterbox_padding();
    BoundingBox transform_bounding_box(const BoundingBox& bbox) const;

    PoseResult compute_pose_from_depth(
        const cv::Mat& depth_image,   // CV_32FC1 meters
        const BoundingBox& bbox_org,  // in original image pixels
        pcl::PointCloud<pcl::PointXYZ>::Ptr annulus_dbg,
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_dbg,
        bool debug_visualize) const;

   private:
    bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::ModelCoefficients::Ptr& coeff,
                       pcl::PointIndices::Ptr& inliers) const;

    Eigen::Vector3f get_ray_direction(const BoundingBox& bbox) const;
    Eigen::Vector3f compute_plane_normal(
        const pcl::ModelCoefficients::Ptr& coeff,
        const Eigen::Vector3f& ray_direction) const;
    Eigen::Vector3f find_ray_plane_intersection(
        const pcl::ModelCoefficients::Ptr& coeff,
        const Eigen::Vector3f& ray_direction,
        const Eigen::Vector3f& ray_origin = Eigen::Vector3f::Zero()) const;
    Eigen::Vector3f shift_point_along_normal(
        const Eigen::Vector3f& intersection_point,
        const Eigen::Vector3f& plane_normal) const;
    Eigen::Matrix3f create_rotation_matrix(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& plane_normal,
        float angle) const;
    // Variant that works entirely in depth frame: color rays are rotated by
    // R_dc = R^T before intersecting the depth-frame plane.
    Eigen::Matrix3f create_rotation_matrix_depth(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& plane_normal,
        float angle,
        const Eigen::Vector3f& ray_origin,
        const Eigen::Matrix3f& R_dc) const;

    ImageProperties color_image_properties_{};
    ImageProperties depth_image_properties_{};
    DepthColorExtrinsic depth_color_extrinsic_{};
    bool has_depth_props_{false};

    int yolo_img_width_;
    int yolo_img_height_;
    float annulus_radius_ratio_;
    float plane_ransac_threshold_;
    int plane_ransac_max_iterations_;
    float valve_handle_offset_;

    double letterbox_scale_factor_{1.0};
    double letterbox_pad_x_{0};
    double letterbox_pad_y_{0};

    mutable Eigen::Vector3f filter_direction_{1, 0, 0};
};

}  // namespace valve_detection
