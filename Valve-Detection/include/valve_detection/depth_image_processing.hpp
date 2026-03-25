#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <utility>
#include <vector>
#include "valve_detection/types.hpp"

namespace valve_detection {

void project_pixel_to_point(const int u,
                            const int v,
                            const float depth,
                            const double fx,
                            const double fy,
                            const double cx,
                            const double cy,
                            pcl::PointXYZ& out);

void extract_annulus_pcl(
    const cv::Mat& depth_image,  // CV_32FC1 meters
    const BoundingBox& bbox,     // in ORIGINAL image pixels
    const ImageProperties& img_props,
    const float annulus_radius_ratio,  // inner radius = outer*ratio
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Iterates depth pixels, back-projects each with depth intrinsics, applies
// the extrinsic transform, then checks whether the resulting color-frame
// projection falls inside the elliptic annulus defined by color_bbox.
// Output points are in the color camera frame.
void extract_annulus_pcl_aligned(
    const cv::Mat& depth_image,     // CV_32FC1 meters, depth frame
    const BoundingBox& color_bbox,  // annulus defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    const float annulus_radius_ratio,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Extracts all valid depth points whose color-frame projection falls inside
// the oriented bounding box.  Output points are in the color camera frame.
void extract_bbox_pcl_aligned(
    const cv::Mat& depth_image,     // CV_32FC1 meters, depth frame
    const BoundingBox& color_bbox,  // OBB defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Projects the 4 corners of the color OBB into depth image space once, fits
// an OBB to those projected corners, then tests depth pixels directly against
// that depth-image OBB — no per-pixel matrix multiply needed.  Output points
// are stored in the depth camera frame.
void extract_bbox_pcl_depth(
    const cv::Mat& depth_image,     // CV_32FC1 meters, depth frame
    const BoundingBox& color_bbox,  // OBB defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Project a color image pixel to depth image coordinates.
// u_c, v_c: pixel coordinates in the color image.
// Z:        depth of the point in the color camera frame (metres).
cv::Point2f project_color_pixel_to_depth(const float u_c,
                                         const float v_c,
                                         const float Z,
                                         const ImageProperties& color_props,
                                         const ImageProperties& depth_props,
                                         const DepthColorExtrinsic& extr);

// Returns undistorted copy of bbox (center_x/y corrected for lens distortion).
BoundingBox undistort_bbox(const BoundingBox& bbox,
                           const CameraIntrinsics& intr);

// Greedy NMS: returns indices of kept detections (max 2).
// scored_boxes: (score, bbox) pairs. Two boxes are duplicates when IoMin
// (intersection / min-area) or IoU exceeds iou_duplicate_threshold.
std::vector<size_t> filter_duplicate_detections(
    const std::vector<std::pair<float, BoundingBox>>& scored_boxes,
    float iou_duplicate_threshold);

}  // namespace valve_detection
