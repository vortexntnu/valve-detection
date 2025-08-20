#ifndef VALVE_POSE_BASE_HPP
#define VALVE_POSE_BASE_HPP

#include "valve_detection/types.hpp"
#include <pcl/point_cloud.h>

namespace valve_detection {
    
class ValvePoseBase {
public:
    ValvePoseBase(int yolo_img_width, int yolo_img_height_,
        float annulus_radius_ratio,
        float plane_ransac_threshold, int plane_ransac_max_iterations,
        float valve_handle_offset);
    virtual ~ValvePoseBase() = default;

    void project_point_to_pixel(
        float x, float y, float z, int& u, int& v) const;

    void calculate_letterbox_properties();

    BoundingBox transform_bounding_box(
        const BoundingBox& bbox) const;

    void set_color_image_properties(
        const ImageProperties& properties) {
        color_image_properties_ = properties;
    }

    /**
    * @brief Segments the dominant plane from a point cloud using RANSAC.
    * @param cloud The input point cloud.
    * @param coefficients The output model coefficients (ax + by + cz + d = 0).
    * @return True if a plane with inliers was found, false otherwise.
    */
    bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::ModelCoefficients::Ptr& coefficients,
        pcl::PointIndices::Ptr& inliers) const;

    Eigen::Vector3f get_ray_direction(const BoundingBox& bbox) const;

    /**
    * @brief Creates a consistent, normalized Eigen::Vector3f from PCL plane coefficients
    * and a reference ray.
    *
    * This function extracts the normal vector (A, B, C) from the plane equation
    * Ax + By + Cz + D = 0, normalizes it, and flips its direction if necessary
    * to ensure it always points in the same general direction as the reference ray.
    *
    * @param coefficients A shared pointer to the pcl::ModelCoefficients object.
    * @param ray_direction A reference to the Eigen::Vector3f representing the ray.
    * @return A normalized Eigen::Vector3f representing the consistent plane normal.
    */
    Eigen::Vector3f create_consistent_plane_normal_with_ray(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& ray_direction) const;


    /**
    * @brief Finds the intersection point of a ray and a plane.
    *
    * The ray is defined by its origin (assumed to be the camera's optical center,
    * i.e., [0, 0, 0]) and a direction vector. The plane is defined by its coefficients
    * Ax + By + Cz + D = 0.
    *
    * @param coefficients A shared pointer to the pcl::ModelCoefficients object.
    * @param ray_direction The normalized Eigen::Vector3f ray direction.
    * @return An Eigen::Vector3f representing the 3D intersection point.
    */
    Eigen::Vector3f find_ray_plane_intersection(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& ray_direction) const;

    /**
    * @brief Shifts a point along a plane normal by a given distance.
    *
    * This function is useful for moving a point from the detected plane's
    * surface to a new location, such as the center of a valve knob.
    * The shift is performed along the vector of the consistent plane normal.
    *
    * @param intersection_point The 3D intersection point on the plane.
    * @param plane_normal The consistent, normalized plane normal vector.
    * @return The new Eigen::Vector3f point after the shift.
    */
    Eigen::Vector3f shift_point_along_normal(
        const Eigen::Vector3f& intersection_point,
        const Eigen::Vector3f& plane_normal) const;

protected:
    ImageProperties color_image_properties_;
    int yolo_img_width_;
    int yolo_img_height_;
    float annulus_radius_ratio_;
    float plane_ransac_threshold_;
    int plane_ransac_max_iterations_;
    float valve_handle_offset_;
    float letterbox_scale_factor_;
    int letterbox_pad_x_;
    int letterbox_pad_y_;
}
} // namespace valve_detection

#endif // VALVE_POSE_BASE_HPP