#ifndef VALVE_POSE_DEPTH_HPP
#define VALVE_POSE_DEPTH_HPP

#include "valve_detection/valve_pose_base.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

namespace valve_detection {

class ValvePoseDepth : public ValvePoseBase {
public:
    ValvePoseDepth(int yolo_img_width, int yolo_img_height_, float annulus_radius_ratio,
        float plane_ransac_threshold, int plane_ransac_max_iterations);
    virtual ~ValvePoseDepth() = default;

    void project_pixel_to_point(int u,
        int v,
        float depth,
        pcl::PointXYZ& point) const;

    void extract_annulus_pcl(
        const cv::Mat& depth_image,
        const BoundingBox& bbox,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

private:
    ImageProperties depth_image_properties_;
    double height_scalar_;
    double width_scalar_;
};

} // namespace valve_detection

#endif // VALVE_POSE_DEPTH_HPP