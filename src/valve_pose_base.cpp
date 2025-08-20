#include "valve_detection/valve_pose_base.hpp"

namespace valve_detection {

ValvePoseBase::ValvePoseBase(int yolo_img_width_, int yolo_img_height_,
        float annulus_radius_ratio,
        float plane_ransac_threshold, int plane_ransac_max_iterations,
        float valve_handle_offset)
    : yolo_img_width_(yolo_img_width_), yolo_img_height_(yolo_img_height_),
    annulus_radius_ratio_(annulus_radius_ratio),
      plane_ransac_threshold_(plane_ransac_threshold),
      plane_ransac_max_iterations_(plane_ransac_max_iterations),
      valve_handle_offset_(valve_handle_offset) {
}

void ValvePoseBase::project_point_to_pixel(float x, float y, float z, int& u, int& v) const {
    if (z <= 0) {
        u = v = -1; // Invalid pixel coordinates
        return;
    }

    double fx = color_image_properties_.intr.fx;
    double fy = color_image_properties_.intr.fy;
    double cx = color_image_properties_.intr.cx;
    double cy = color_image_properties_.intr.cy;

    u = static_cast<int>((x * fx / z) + cx);
    v = static_cast<int>((y * fy / z) + cy);
}

void ValvePoseBase::calculate_letterbox_padding() {
    int org_image_width = color_image_properties_.dim.x;
    int org_image_height = color_image_properties_.dim.y;

    letterbox_scale_factor_ = std::min(
        static_cast<double>(yolo_input_width) / original_width,
        static_cast<double>(yolo_input_height) / original_height);

    double resized_width = original_width * letterbox_scale_factor_;
    double resized_height = original_height * letterbox_scale_factor_;

    letterbox_pad_x_ = (yolo_input_width - resized_width) / 2.0;
    letterbox_pad_y_ = (yolo_input_height - resized_height) / 2.0;
}

BoundingBox ValvePoseBase::transform_bounding_box(const BoundingBox& bbox) const {
    BoundingBox transformed_bbox = bbox;

    transformed_bbox.center.position.x = (bbox.center_x - letterbox_pad_x_) / letterbox_scale_factor_;
    transformed_bbox.center.position.y = (bbox.center_y - letterbox_pad_y_) / letterbox_scale_factor_;
    transformed_bbox.size_x /= letterbox_scale_factor_;
    transformed_bbox.size_y /= letterbox_scale_factor_;

    return transformed_bbox;
}

bool ValvePoseBase::segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::ModelCoefficients::Ptr& coefficients,
    pcl::PointIndices::Ptr& inliers) const {

    if (cloud->points.empty()) {
    return false;
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_ransac_threshold_);
    seg.setMaxIterations(plane_ransac_max_iterations_);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    return !inliers->indices.empty();
}

Eigen::Vector3f ValvePoseBase::get_ray_direction(const BoundingBox& bbox) const {
    float u = bbox.center_x;
    float v = bbox.center_y;

    float fx = color_image_properties_.intr.fx;
    float fy = color_image_properties_.intr.fy;
    float cx = color_image_properties_.intr.cx;
    float cy = color_image_properties_.intr.cy;

    float xc = (u - cx) / fx;
    float yc = (v - cy) / fy;

    Eigen::Vector3f ray_direction;
    ray_direction << xc, yc, 1.0f;

    return ray_direction.normalized();
}

Eigen::Vector3f ValvePoseBase::compute_plane_normal(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction) {

    Eigen::Vector3f normal_vector;

    if (!coefficients || coefficients->values.size() < 3) {
        return Eigen::Vector3f::Zero();
    }

    normal_vector << coefficients->values[0], coefficients->values[1], coefficients->values[2];
    
    normal_vector.normalize();

    if (normal_vector.dot(ray_direction) > 0.0) {
        normal_vector = -normal_vector;
    }

    return normal_vector;
}

/**
 * @brief Finds the intersection point of a ray and a plane.
 *
 * The ray is defined by its origin (The camera's optical center,
 * i.e., [0, 0, 0]) and a direction vector. The plane is defined by its coefficients
 * Ax + By + Cz + D = 0.
 *
 * @param coefficients A shared pointer to the pcl::ModelCoefficients object.
 * @param ray_direction The normalized Eigen::Vector3f ray direction.
 * @return An Eigen::Vector3f representing the 3D intersection point.
 */
 Eigen::Vector3f ValvePoseBase::find_ray_plane_intersection(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction) const {

    // Plane equation: Ax + By + Cz + D = 0
    // Ray equation:   P = O + t * d, where O is origin [0,0,0], P is point, t is scalar, d is direction.
    // Substituting the ray equation into the plane equation:
    // A*(O.x + t*d.x) + B*(O.y + t*d.y) + C*(O.z + t*d.z) + D = 0
    // Since O is [0,0,0], this simplifies to:
    // A*t*d.x + B*t*d.y + C*t*d.z + D = 0
    // t * (A*d.x + B*d.y + C*d.z) = -D
    // t = -D / (A*d.x + B*d.y + C*d.z)

    if (!coefficients || coefficients->values.size() < 4) {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f plane_normal;
    plane_normal << coefficients->values[0], coefficients->values[1], coefficients->values[2];
    float D = coefficients->values[3];

    float normal_dot_ray = plane_normal.dot(ray_direction);

    if (std::abs(normal_dot_ray) < 1e-6) {
        // The ray is parallel or contained within the plane.
        return Eigen::Vector3f::Zero();
    }

    float t = -D / normal_dot_ray;

    Eigen::Vector3f intersection_point = t * ray_direction;

    return intersection_point;
}

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
 Eigen::Vector3f ValvePoseBase::shift_point_along_normal(
    const Eigen::Vector3f& intersection_point,
    const Eigen::Vector3f& plane_normal) const {
    
    return intersection_point + (plane_normal * valve_handle_offset_);
}


} // namespace valve_detection