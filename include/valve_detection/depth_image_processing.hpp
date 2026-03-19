#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include "valve_detection/types.hpp"

namespace valve_detection {

void project_pixel_to_point(int u,
                            int v,
                            float depth,
                            double fx,
                            double fy,
                            double cx,
                            double cy,
                            pcl::PointXYZ& out);

void extract_annulus_pcl(
    const cv::Mat& depth_image,  // CV_32FC1 meters
    const BoundingBox& bbox,     // in ORIGINAL image pixels
    const ImageProperties& img_props,
    float annulus_radius_ratio,  // inner radius = outer*ratio
    pcl::PointCloud<pcl::PointXYZ>::Ptr& out);

// Hardcoded depth-to-color extrinsic for Intel RealSense D555.
// Values from the camera URDF / factory calibration.
// Verify with: ros2 topic echo /realsense/extrinsics/depth_to_color
// or the URDF at https://github.com/IntelRealSense/librealsense/issues/14577
DepthColorExtrinsic d555_depth_to_color_extrinsic();

// Like extract_annulus_pcl but with proper depth-to-color alignment.
// Iterates depth pixels, back-projects with depth intrinsics, applies the
// extrinsic transform, then checks whether the resulting color-frame
// projection falls inside the annulus.  Output points are in the color
// camera frame.
void extract_annulus_pcl_aligned(
    const cv::Mat& depth_image,     // CV_32FC1 meters, depth frame
    const BoundingBox& color_bbox,  // annulus defined in color pixels
    const ImageProperties& color_props,
    const ImageProperties& depth_props,
    const DepthColorExtrinsic& extrinsic,
    float annulus_radius_ratio,
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

// Project a color image pixel to depth image coordinates.
// u_c, v_c: pixel coordinates in the color image.
// Z:        depth of the point in the color camera frame (metres).
cv::Point2f project_color_pixel_to_depth(float u_c,
                                         float v_c,
                                         float Z,
                                         const ImageProperties& color_props,
                                         const ImageProperties& depth_props,
                                         const DepthColorExtrinsic& extr);

}  // namespace valve_detection
