#include "valve_detection/depth_image_processing.hpp"

namespace valve_detection {

void project_pixel_to_point(int u,
                            int v,
                            float depth,
                            double fx,
                            double fy,
                            double cx,
                            double cy,
                            pcl::PointXYZ& point) {
    if (depth <= 0 || depth == std::numeric_limits<float>::infinity() ||
        std::isnan(depth)) {
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    point.x = (u - cx) * depth / fx;
    point.y = (v - cy) * depth / fy;
    point.z = depth;
}

void extract_annulus_pcl(const cv::Mat& depth_image,
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

    int upper_bound_size = static_cast<int>(
        M_PI *
        ((outer_radius_x * outer_radius_y - inner_radius_x * inner_radius_y) +
         1));

    cloud->clear();
    cloud->reserve(upper_bound_size);

    for (int y = static_cast<int>(-outer_radius_y);
         y <= static_cast<int>(outer_radius_y); ++y) {
        for (int x = static_cast<int>(-outer_radius_x);
             x <= static_cast<int>(outer_radius_x); ++x) {
            float dx_scaled_outer = x / outer_radius_x;
            float dy_scaled_outer = y / outer_radius_y;
            bool is_inside_outer_ellipse =
                (dx_scaled_outer * dx_scaled_outer +
                 dy_scaled_outer * dy_scaled_outer) <= 1.0f;

            float dx_scaled_inner = x / inner_radius_x;
            float dy_scaled_inner = y / inner_radius_y;
            bool is_outside_inner_ellipse =
                (dx_scaled_inner * dx_scaled_inner +
                 dy_scaled_inner * dy_scaled_inner) > 1.0f;

            if (is_inside_outer_ellipse && is_outside_inner_ellipse) {
                int u = static_cast<int>(center_x + x);
                int v = static_cast<int>(center_y + y);

                if (u >= 0 && u < depth_image.cols && v >= 0 &&
                    v < depth_image.rows) {
                    float depth_value = depth_image.at<float>(v, u);
                    pcl::PointXYZ point;
                    project_pixel_to_point(
                        u, v, depth_value, image_properties.intr.fx,
                        image_properties.intr.fy, image_properties.intr.cx,
                        image_properties.intr.cy, point);

                    if (!std::isnan(point.x) && !std::isnan(point.y) &&
                        !std::isnan(point.z)) {
                        cloud->points.push_back(point);
                    }
                }
            }
        }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
}

}  // namespace valve_detection
