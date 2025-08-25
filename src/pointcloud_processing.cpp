#include "valve_detection/pointcloud_processing.hpp"

namespace valve_detection {

void extract_annulus_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                         const BoundingBox& bbox,
                         const ImageProperties& image_properties,
                         float annulus_radius_ratio,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    float center_x = bbox.center_x;
    float center_y = bbox.center_y;
    float outer_radius_x = bbox.size_x / 2.0f;
    float outer_radius_y = bbox.size_y / 2.0f;

    float inner_radius_x = outer_radius_x * annulus_radius_ratio;
    float inner_radius_y = outer_radius_y * annulus_radius_ratio;

    double fx = image_properties.intr.fx;
    double fy = image_properties.intr.fy;
    double cx = image_properties.intr.cx;
    double cy = image_properties.intr.cy;

    cloud->clear();
    cloud->points.reserve(input_cloud->points.size());

    for (const auto& point_in : input_cloud->points) {
        if (std::isnan(point_in.z)) {
            continue;
        }

        int u = static_cast<int>((point_in.x * fx / point_in.z) + cx);
        int v = static_cast<int>((point_in.y * fy / point_in.z) + cy);

        if (u >= center_x - outer_radius_x && u <= center_x + outer_radius_x &&
            v >= center_y - outer_radius_y && v <= center_y + outer_radius_y) {
            float dx_scaled_outer = (u - center_x) / outer_radius_x;
            float dy_scaled_outer = (v - center_y) / outer_radius_y;
            bool is_inside_outer_ellipse =
                (dx_scaled_outer * dx_scaled_outer +
                 dy_scaled_outer * dy_scaled_outer) <= 1.0f;

            float dx_scaled_inner = (u - center_x) / inner_radius_x;
            float dy_scaled_inner = (v - center_y) / inner_radius_y;
            bool is_outside_inner_ellipse =
                (dx_scaled_inner * dx_scaled_inner +
                 dy_scaled_inner * dy_scaled_inner) > 1.0f;

            if (is_inside_outer_ellipse && is_outside_inner_ellipse) {
                cloud->points.push_back(point_in);
            }
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
}

}  // namespace valve_detection
