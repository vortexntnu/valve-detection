// depth_image_processing.cpp
// Depth-to-3D back-projection, point cloud extraction, and color-to-depth pixel
// projection.
#include "valve_detection/depth_image_processing.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace valve_detection {

// Back-projects a depth pixel (u, v, depth) to a 3D point using camera
// intrinsics.
void project_pixel_to_point(int u,
                            int v,
                            float depth,
                            double fx,
                            double fy,
                            double cx,
                            double cy,
                            pcl::PointXYZ& out) {
    if (depth <= 0.0f || std::isnan(depth) || std::isinf(depth)) {
        out.x = out.y = out.z = std::numeric_limits<float>::quiet_NaN();
        return;
    }
    out.x = static_cast<float>((u - cx) * depth / fx);
    out.y = static_cast<float>((v - cy) * depth / fy);
    out.z = depth;
}

// Extracts depth points that fall inside an elliptic annulus around the bbox
// center (no alignment).
void extract_annulus_pcl(const cv::Mat& depth_image,
                         const BoundingBox& bbox,
                         const ImageProperties& img_props,
                         float annulus_radius_ratio,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    const float cx = bbox.center_x;
    const float cy = bbox.center_y;
    const float outer_rx = bbox.size_x * 0.5f;
    const float outer_ry = bbox.size_y * 0.5f;

    if (outer_rx < 2.0f || outer_ry < 2.0f)
        return;

    const float inner_rx = outer_rx * annulus_radius_ratio;
    const float inner_ry = outer_ry * annulus_radius_ratio;

    const int u0 = static_cast<int>(std::floor(cx - outer_rx));
    const int u1 = static_cast<int>(std::ceil(cx + outer_rx));
    const int v0 = static_cast<int>(std::floor(cy - outer_ry));
    const int v1 = static_cast<int>(std::ceil(cy + outer_ry));

    for (int v = v0; v <= v1; ++v) {
        if (v < 0 || v >= depth_image.rows)
            continue;
        for (int u = u0; u <= u1; ++u) {
            if (u < 0 || u >= depth_image.cols)
                continue;

            const float dxo = (u - cx) / outer_rx;
            const float dyo = (v - cy) / outer_ry;
            const bool inside_outer = (dxo * dxo + dyo * dyo) <= 1.0f;

            const float dxi = (u - cx) / inner_rx;
            const float dyi = (v - cy) / inner_ry;
            const bool outside_inner = (dxi * dxi + dyi * dyi) > 1.0f;

            if (!inside_outer || !outside_inner)
                continue;

            const float z = depth_image.at<float>(v, u);
            pcl::PointXYZ p;
            project_pixel_to_point(u, v, z, img_props.intr.fx,
                                   img_props.intr.fy, img_props.intr.cx,
                                   img_props.intr.cy, p);

            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
                out->points.push_back(p);
            }
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Returns hardcoded depth-to-color extrinsic for Intel RealSense D555.
// Verify with: ros2 topic echo /realsense/extrinsics/depth_to_color
DepthColorExtrinsic d555_depth_to_color_extrinsic() {
    DepthColorExtrinsic extr;
    // Both depth and color optical frames share the same body-to-optical
    // rotation, so R is identity.
    extr.R = Eigen::Matrix3f::Identity();
    // t = depth origin expressed in color optical frame.
    // Color is 0.059 m in -y (rightward) of depth in body frame, so depth is
    // +0.059 m in y (leftward) of color. Converting to optical frame:
    //   t_optical = R_body_to_optical * [0, +0.059, 0]
    //             = [[0,-1,0],[0,0,-1],[1,0,0]] * [0, 0.059, 0]
    //             = [-0.059, 0, 0]
    extr.t = Eigen::Vector3f(-0.059f, 0.0f, 0.0f);
    return extr;
}

// Like extract_annulus_pcl but transforms depth pixels into the color frame
// before the annulus test.
void extract_annulus_pcl_aligned(const cv::Mat& depth_image,
                                 const BoundingBox& color_bbox,
                                 const ImageProperties& color_props,
                                 const ImageProperties& depth_props,
                                 const DepthColorExtrinsic& extr,
                                 float annulus_radius_ratio,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    const float cx_c = color_bbox.center_x;
    const float cy_c = color_bbox.center_y;
    const float outer_rx = color_bbox.size_x * 0.5f;
    const float outer_ry = color_bbox.size_y * 0.5f;

    if (outer_rx < 2.0f || outer_ry < 2.0f)
        return;

    const float inner_rx = outer_rx * annulus_radius_ratio;
    const float inner_ry = outer_ry * annulus_radius_ratio;

    // Approximate scale between depth and color focal lengths to define a
    // coarse search region in depth-image coordinates.  A 30-pixel margin
    // accounts for the lateral offset introduced by the extrinsic.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    constexpr int kMargin = 30;

    const int u0_d =
        std::max(0, static_cast<int>((cx_c - outer_rx) * scale) - kMargin);
    const int u1_d =
        std::min(depth_image.cols - 1,
                 static_cast<int>((cx_c + outer_rx) * scale) + kMargin);
    const int v0_d =
        std::max(0, static_cast<int>((cy_c - outer_ry) * scale) - kMargin);
    const int v1_d =
        std::min(depth_image.rows - 1,
                 static_cast<int>((cy_c + outer_ry) * scale) + kMargin);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project using depth intrinsics.
            Eigen::Vector3f P_d;
            P_d.x() = static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                         depth_props.intr.fx);
            P_d.y() = static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                         depth_props.intr.fy);
            P_d.z() = z_d;

            // Transform into color camera frame.
            const Eigen::Vector3f P_c = extr.R * P_d + extr.t;
            if (P_c.z() <= 0.0f)
                continue;

            // Project onto color image plane.
            const float u_c = static_cast<float>(
                color_props.intr.fx * P_c.x() / P_c.z() + color_props.intr.cx);
            const float v_c = static_cast<float>(
                color_props.intr.fy * P_c.y() / P_c.z() + color_props.intr.cy);

            // Elliptic annulus test in color-image space.
            const float dxo = (u_c - cx_c) / outer_rx;
            const float dyo = (v_c - cy_c) / outer_ry;
            if (dxo * dxo + dyo * dyo > 1.0f)
                continue;  // outside outer ellipse

            const float dxi = (u_c - cx_c) / inner_rx;
            const float dyi = (v_c - cy_c) / inner_ry;
            if (dxi * dxi + dyi * dyi <= 1.0f)
                continue;  // inside inner ellipse

            pcl::PointXYZ p;
            p.x = P_c.x();
            p.y = P_c.y();
            p.z = P_c.z();
            out->points.push_back(p);
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Extracts depth points whose color-frame projection falls inside the oriented
// bounding box.
void extract_bbox_pcl_aligned(const cv::Mat& depth_image,
                              const BoundingBox& color_bbox,
                              const ImageProperties& color_props,
                              const ImageProperties& depth_props,
                              const DepthColorExtrinsic& extr,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Pre-compute the OBB axes in color-image space for a fast rotated-rect
    // test.
    const float cos_t = std::cos(color_bbox.theta);
    const float sin_t = std::sin(color_bbox.theta);
    const float half_w = color_bbox.size_x * 0.5f;
    const float half_h = color_bbox.size_y * 0.5f;

    // Approximate search region in depth-image space.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    const float r = std::sqrt(half_w * half_w + half_h * half_h);
    constexpr int kMargin = 30;

    const int u0_d = std::max(
        0, static_cast<int>((color_bbox.center_x - r) * scale) - kMargin);
    const int u1_d =
        std::min(depth_image.cols - 1,
                 static_cast<int>((color_bbox.center_x + r) * scale) + kMargin);
    const int v0_d = std::max(
        0, static_cast<int>((color_bbox.center_y - r) * scale) - kMargin);
    const int v1_d =
        std::min(depth_image.rows - 1,
                 static_cast<int>((color_bbox.center_y + r) * scale) + kMargin);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project to depth camera frame.
            Eigen::Vector3f P_d;
            P_d.x() = static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                         depth_props.intr.fx);
            P_d.y() = static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                         depth_props.intr.fy);
            P_d.z() = z_d;

            // Transform to color camera frame.
            const Eigen::Vector3f P_c = extr.R * P_d + extr.t;
            if (P_c.z() <= 0.0f)
                continue;

            // Project onto color image plane.
            const float u_c = static_cast<float>(
                color_props.intr.fx * P_c.x() / P_c.z() + color_props.intr.cx);
            const float v_c = static_cast<float>(
                color_props.intr.fy * P_c.y() / P_c.z() + color_props.intr.cy);

            // Rotate into the OBB local frame and test against the
            // half-extents.
            const float dx = u_c - color_bbox.center_x;
            const float dy = v_c - color_bbox.center_y;
            const float local_x = cos_t * dx + sin_t * dy;
            const float local_y = -sin_t * dx + cos_t * dy;

            if (std::abs(local_x) > half_w || std::abs(local_y) > half_h)
                continue;

            out->points.push_back({P_c.x(), P_c.y(), P_c.z()});
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Projects a color image pixel to depth image coordinates using the full
// intrinsic + extrinsic pipeline.
cv::Point2f project_color_pixel_to_depth(float u_c,
                                         float v_c,
                                         float Z,
                                         const ImageProperties& color_props,
                                         const ImageProperties& depth_props,
                                         const DepthColorExtrinsic& extr) {
    // Back-project color pixel → 3-D point in color camera frame.
    const float Xc = (u_c - static_cast<float>(color_props.intr.cx)) * Z /
                     static_cast<float>(color_props.intr.fx);
    const float Yc = (v_c - static_cast<float>(color_props.intr.cy)) * Z /
                     static_cast<float>(color_props.intr.fy);
    const Eigen::Vector3f Pc(Xc, Yc, Z);

    // Transform to depth camera frame:  P_depth = R^T * (P_color - t)
    const Eigen::Vector3f Pd = extr.R.transpose() * (Pc - extr.t);

    if (Pd.z() <= 0.0f)
        return {u_c, v_c};  // degenerate – return original

    // Project into depth image.
    const float u_d =
        static_cast<float>(depth_props.intr.fx) * Pd.x() / Pd.z() +
        static_cast<float>(depth_props.intr.cx);
    const float v_d =
        static_cast<float>(depth_props.intr.fy) * Pd.y() / Pd.z() +
        static_cast<float>(depth_props.intr.cy);
    return {u_d, v_d};
}

}  // namespace valve_detection
