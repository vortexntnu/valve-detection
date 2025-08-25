#ifndef VALVE_POSE_PCL_HPP
#define VALVE_POSE_PCL_HPP

#include "valve_detection/types.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <limits>

namespace valve_detection {

/**
 * @brief Extracts a 3D point cloud representing the annulus (valve rim) from an
 * input point cloud.
 *
 * This function selects points that correspond to the annulus region of a
 * bounding box in image space, based on the camera intrinsics and the specified
 * radius ratio.
 *
 * @param input_cloud The input point cloud (typically from a depth camera or
 * LIDAR).
 * @param bbox The bounding box around the valve in image coordinates.
 * @param image_properties Camera intrinsics and image dimensions for
 * projection.
 * @param annulus_radius_ratio Fraction of the bounding box considered as the
 * annulus (0.0–1.0).
 * @param[out] cloud The resulting point cloud containing the 3D points of the
 * annulus.
 */
void extract_annulus_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                         const BoundingBox& bbox,
                         const ImageProperties& image_properties,
                         float annulus_radius_ratio,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

}  // namespace valve_detection

#endif  // VALVE_POSE_PCL_HPP
