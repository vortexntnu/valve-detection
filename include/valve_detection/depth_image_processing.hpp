#ifndef VALVE_POSE_DEPTH_HPP
#define VALVE_POSE_DEPTH_HPP

#include "valve_detection/types.hpp"

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>
#include <opencv2/core/mat.hpp>

namespace valve_detection {

/**
 * @brief Projects a 2D pixel with depth into 3D camera coordinates.
 *
 * @param u The x-coordinate of the pixel in the image.
 * @param v The y-coordinate of the pixel in the image.
 * @param depth The depth value at the pixel (in meters or the same units as
 * intrinsics).
 * @param fx Focal length along x-axis.
 * @param fy Focal length along y-axis.
 * @param cx Principal point x-coordinate.
 * @param cy Principal point y-coordinate.
 * @param[out] point The resulting 3D point in camera coordinates.
 */
void project_pixel_to_point(int u,
                            int v,
                            float depth,
                            double fx,
                            double fy,
                            double cx,
                            double cy,
                            pcl::PointXYZ& point);

/**
 * @brief Extracts a 3D point cloud representing the annulus (valve rim) from a
 * depth image.
 *
 * The function selects pixels within the annulus region of a bounding box and
 * projects them into 3D points using the camera intrinsics.
 *
 * @param depth_image The input depth image (CV_32FC1 or similar).
 * @param bbox The bounding box around the valve in the image.
 * @param image_properties Camera intrinsics and image dimensions.
 * @param annulus_radius_ratio Fraction of the bounding box considered as the
 * annulus (0.0–1.0).
 * @param[out] cloud The resulting point cloud containing the 3D points of the
 * annulus.
 */
void extract_annulus_pcl(const cv::Mat& depth_image,
                         const BoundingBox& bbox,
                         const ImageProperties& image_properties,
                         float annulus_radius_ratio,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

}  // namespace valve_detection

#endif  // VALVE_POSE_DEPTH_HPP
