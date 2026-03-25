// depth_image_processing.cpp
// Depth-to-3D back-projection, point cloud extraction, and color-to-depth pixel
// projection.
#include "valve_detection/depth_image_processing.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace valve_detection {

// Back-projects a depth pixel (u, v, depth) to a 3D point using camera
// intrinsics.
void project_pixel_to_point(const int u,
                            const int v,
                            const float depth,
                            const double fx,
                            const double fy,
                            const double cx,
                            const double cy,
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
                         const float annulus_radius_ratio,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    const float cx = bbox.center_x;
    const float cy = bbox.center_y;
    // Outer ellipse half-extents match the bounding box.
    const float outer_rx = bbox.size_x * 0.5f;
    const float outer_ry = bbox.size_y * 0.5f;

    // Require at least 4 pixels in each dimension (2 px half-extent) before
    // sampling — smaller boxes contain no usable depth points.
    if (outer_rx < 2.0f || outer_ry < 2.0f)
        return;

    // Inner ellipse is scaled down by annulus_radius_ratio, creating a ring
    // that avoids the central hub and samples only the valve rim.
    const float inner_rx = outer_rx * annulus_radius_ratio;
    const float inner_ry = outer_ry * annulus_radius_ratio;

    // Bounding rectangle of the outer ellipse in pixel space.
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

            // Normalise pixel offset by the outer half-extents; the point is
            // inside the outer ellipse when the sum of squares ≤ 1.
            const float dxo = (u - cx) / outer_rx;
            const float dyo = (v - cy) / outer_ry;
            const bool inside_outer = (dxo * dxo + dyo * dyo) <= 1.0f;

            // Same test for the inner ellipse; keep only points outside it
            // to form the annular ring.
            const float dxi = (u - cx) / inner_rx;
            const float dyi = (v - cy) / inner_ry;
            const bool outside_inner = (dxi * dxi + dyi * dyi) > 1.0f;

            if (!inside_outer || !outside_inner)
                continue;

            // Back-project the depth pixel to a 3-D point in camera space.
            const float z = depth_image.at<float>(v, u);
            pcl::PointXYZ p;
            project_pixel_to_point(u, v, z, img_props.intr.fx,
                                   img_props.intr.fy, img_props.intr.cx,
                                   img_props.intr.cy, p);

            // Discard invalid points (zero/NaN depth produces NaN coords).
            if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
                out->points.push_back(p);
            }
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Iterates depth pixels, back-projects each with depth intrinsics, applies
// the extrinsic transform, and checks whether the resulting color-frame
// projection falls inside the elliptic annulus defined by color_bbox.
// Output points are in the color camera frame.
void extract_annulus_pcl_aligned(const cv::Mat& depth_image,
                                 const BoundingBox& color_bbox,
                                 const ImageProperties& color_props,
                                 const ImageProperties& depth_props,
                                 const DepthColorExtrinsic& extr,
                                 const float annulus_radius_ratio,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Annulus defined in color-image coordinates.
    const float cx_c = color_bbox.center_x;
    const float cy_c = color_bbox.center_y;
    const float outer_rx = color_bbox.size_x * 0.5f;
    const float outer_ry = color_bbox.size_y * 0.5f;

    // Require at least 4 pixels in each dimension (2 px half-extent) before
    // sampling — smaller boxes contain no usable depth points.
    if (outer_rx < 2.0f || outer_ry < 2.0f)
        return;

    // Inner ellipse scaled down by annulus_radius_ratio, forming the ring.
    const float inner_rx = outer_rx * annulus_radius_ratio;
    const float inner_ry = outer_ry * annulus_radius_ratio;

    // The depth and color images have different resolutions and focal lengths.
    // Scale color-space bbox bounds into depth-image coordinates to get a
    // coarse candidate region, then add a margin to cover the lateral shift
    // introduced by the depth-to-color translation.
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

            // Back-project depth pixel to 3-D point in depth camera frame.
            Eigen::Vector3f P_d;
            P_d.x() = static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                         depth_props.intr.fx);
            P_d.y() = static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                         depth_props.intr.fy);
            P_d.z() = z_d;

            // Apply extrinsic to move the point into the color camera frame,
            // where the annulus is defined.
            const Eigen::Vector3f P_c = extr.R * P_d + extr.t;
            if (P_c.z() <= 0.0f)
                continue;

            // Project the color-frame point onto the color image plane.
            const float u_c = static_cast<float>(
                color_props.intr.fx * P_c.x() / P_c.z() + color_props.intr.cx);
            const float v_c = static_cast<float>(
                color_props.intr.fy * P_c.y() / P_c.z() + color_props.intr.cy);

            // Normalise offset by outer half-extents; keep only points inside
            // the outer ellipse (sum of squares ≤ 1).
            const float dxo = (u_c - cx_c) / outer_rx;
            const float dyo = (v_c - cy_c) / outer_ry;
            if (dxo * dxo + dyo * dyo > 1.0f)
                continue;  // outside outer ellipse

            // Discard points inside the inner ellipse to form the ring.
            const float dxi = (u_c - cx_c) / inner_rx;
            const float dyi = (v_c - cy_c) / inner_ry;
            if (dxi * dxi + dyi * dyi <= 1.0f)
                continue;  // inside inner ellipse

            // Store the point in the color camera frame.
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

// Projects the 4 corners of the color OBB into depth image space once, fits
// an OBB to those projected corners, then tests depth pixels directly against
// that depth-image OBB — no per-pixel matrix multiply needed.  Output points
// are stored in the depth camera frame.
void extract_bbox_pcl_depth(const cv::Mat& depth_image,
                            const BoundingBox& color_bbox,
                            const ImageProperties& color_props,
                            const ImageProperties& depth_props,
                            const DepthColorExtrinsic& extr,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
    out->clear();

    // Sample a representative depth near the bbox center to project the color
    // OBB corners into depth image space.  The valve is roughly flat, so a
    // single Z gives a good approximation for all 4 corners.
    const float scale =
        (depth_props.intr.fx > 0.0 && color_props.intr.fx > 0.0)
            ? static_cast<float>(depth_props.intr.fx / color_props.intr.fx)
            : 1.0f;
    const int u_c_d = std::clamp(static_cast<int>(color_bbox.center_x * scale),
                                 0, depth_image.cols - 1);
    const int v_c_d = std::clamp(static_cast<int>(color_bbox.center_y * scale),
                                 0, depth_image.rows - 1);

    float Z_est = 0.0f;
    constexpr int kSampleRadius = 5;
    for (int dv = -kSampleRadius; dv <= kSampleRadius && Z_est <= 0.0f; ++dv) {
        for (int du = -kSampleRadius; du <= kSampleRadius && Z_est <= 0.0f;
             ++du) {
            const int u = std::clamp(u_c_d + du, 0, depth_image.cols - 1);
            const int v = std::clamp(v_c_d + dv, 0, depth_image.rows - 1);
            const float z = depth_image.at<float>(v, u);
            if (z > 0.0f && !std::isnan(z) && !std::isinf(z))
                Z_est = z;
        }
    }
    if (Z_est <= 0.0f)
        return;

    // Project the 4 color OBB corners into depth image coordinates.
    const float angle_deg =
        color_bbox.theta * 180.0f / static_cast<float>(M_PI);
    cv::RotatedRect color_rrect(
        cv::Point2f(color_bbox.center_x, color_bbox.center_y),
        cv::Size2f(color_bbox.size_x, color_bbox.size_y), angle_deg);
    cv::Point2f color_corners[4];
    color_rrect.points(color_corners);

    std::vector<cv::Point2f> depth_corners(4);
    for (int i = 0; i < 4; ++i) {
        depth_corners[i] =
            project_color_pixel_to_depth(color_corners[i].x, color_corners[i].y,
                                         Z_est, color_props, depth_props, extr);
    }

    // Fit a rotated rect to the 4 projected corners in depth image space.
    const cv::RotatedRect depth_rrect = cv::minAreaRect(depth_corners);
    const float angle_d = depth_rrect.angle * static_cast<float>(M_PI) / 180.0f;
    const float cos_d = std::cos(angle_d);
    const float sin_d = std::sin(angle_d);
    const float half_w_d = depth_rrect.size.width * 0.5f;
    const float half_h_d = depth_rrect.size.height * 0.5f;
    const float cx_d = depth_rrect.center.x;
    const float cy_d = depth_rrect.center.y;

    // Bounding rectangle of the depth OBB.
    const float r_d = std::sqrt(half_w_d * half_w_d + half_h_d * half_h_d);
    const int u0_d = std::max(0, static_cast<int>(cx_d - r_d) - 1);
    const int u1_d =
        std::min(depth_image.cols - 1, static_cast<int>(cx_d + r_d) + 1);
    const int v0_d = std::max(0, static_cast<int>(cy_d - r_d) - 1);
    const int v1_d =
        std::min(depth_image.rows - 1, static_cast<int>(cy_d + r_d) + 1);

    for (int v_d = v0_d; v_d <= v1_d; ++v_d) {
        for (int u_d = u0_d; u_d <= u1_d; ++u_d) {
            // Test against the depth-image OBB — no matrix multiply.
            const float dx = u_d - cx_d;
            const float dy = v_d - cy_d;
            if (std::abs(cos_d * dx + sin_d * dy) > half_w_d ||
                std::abs(-sin_d * dx + cos_d * dy) > half_h_d)
                continue;

            const float z_d = depth_image.at<float>(v_d, u_d);
            if (z_d <= 0.0f || std::isnan(z_d) || std::isinf(z_d))
                continue;

            // Back-project to 3D depth frame.
            out->points.push_back(
                {static_cast<float>((u_d - depth_props.intr.cx) * z_d /
                                    depth_props.intr.fx),
                 static_cast<float>((v_d - depth_props.intr.cy) * z_d /
                                    depth_props.intr.fy),
                 z_d});
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = false;
}

// Projects a color image pixel to depth image coordinates using the full
// intrinsic + extrinsic pipeline.
cv::Point2f project_color_pixel_to_depth(const float u_c,
                                         const float v_c,
                                         const float Z,
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

// Corrects the bbox center for lens distortion using the given intrinsics.
BoundingBox undistort_bbox(const BoundingBox& bbox,
                           const CameraIntrinsics& intr) {
    const cv::Mat K = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.cx, 0,
                       intr.fy, intr.cy, 0, 0, 1);
    const cv::Mat D = (cv::Mat_<double>(5, 1) << intr.dist[0], intr.dist[1],
                       intr.dist[2], intr.dist[3], intr.dist[4]);
    // Build a RotatedRect and extract all 4 corners.
    const float angle_deg = bbox.theta * 180.0f / static_cast<float>(M_PI);
    cv::RotatedRect rrect(cv::Point2f(bbox.center_x, bbox.center_y),
                          cv::Size2f(bbox.size_x, bbox.size_y), angle_deg);
    cv::Point2f corners[4];
    rrect.points(corners);

    // Undistort all 4 corners.
    std::vector<cv::Point2f> pts(corners, corners + 4);
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(pts, undistorted, K, D, cv::noArray(), K);

    // Refit an OBB to the undistorted corners.
    cv::RotatedRect fitted = cv::minAreaRect(undistorted);

    BoundingBox result = bbox;
    result.center_x = fitted.center.x;
    result.center_y = fitted.center.y;
    result.size_x = fitted.size.width;
    result.size_y = fitted.size.height;
    result.theta = fitted.angle * static_cast<float>(M_PI) / 180.0f;
    return result;
}

// Greedy NMS: sorts by score descending, keeps at most 2 non-overlapping boxes.
std::vector<size_t> filter_duplicate_detections(
    const std::vector<std::pair<float, BoundingBox>>& scored_boxes,
    float iou_duplicate_threshold) {
    const size_t n = scored_boxes.size();
    if (n == 0)
        return {};

    std::vector<std::pair<float, size_t>> order;
    order.reserve(n);
    for (size_t i = 0; i < n; ++i)
        order.emplace_back(scored_boxes[i].first, i);
    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<size_t> kept;
    std::vector<bool> suppressed(n, false);

    for (size_t si = 0; si < order.size() && kept.size() < 2; ++si) {
        const size_t i = order[si].second;
        if (suppressed[i])
            continue;

        kept.push_back(i);

        const BoundingBox& bi = scored_boxes[i].second;
        const float ai = bi.size_x * bi.size_y;
        const float bx1i = bi.center_x - bi.size_x * 0.5f;
        const float by1i = bi.center_y - bi.size_y * 0.5f;
        const float bx2i = bi.center_x + bi.size_x * 0.5f;
        const float by2i = bi.center_y + bi.size_y * 0.5f;

        for (size_t sj = si + 1; sj < order.size(); ++sj) {
            const size_t j = order[sj].second;
            if (suppressed[j])
                continue;

            const BoundingBox& bj = scored_boxes[j].second;
            const float aj = bj.size_x * bj.size_y;
            const float bx1j = bj.center_x - bj.size_x * 0.5f;
            const float by1j = bj.center_y - bj.size_y * 0.5f;
            const float bx2j = bj.center_x + bj.size_x * 0.5f;
            const float by2j = bj.center_y + bj.size_y * 0.5f;

            const float ix1 = std::max(bx1i, bx1j);
            const float iy1 = std::max(by1i, by1j);
            const float ix2 = std::min(bx2i, bx2j);
            const float iy2 = std::min(by2i, by2j);

            if (ix2 <= ix1 || iy2 <= iy1)
                continue;

            const float inter = (ix2 - ix1) * (iy2 - iy1);
            const float iou = inter / (ai + aj - inter);
            const float iom = inter / std::min(ai, aj);

            if (iou > iou_duplicate_threshold || iom > 0.7f)
                suppressed[j] = true;
        }
    }

    return kept;
}

}  // namespace valve_detection
