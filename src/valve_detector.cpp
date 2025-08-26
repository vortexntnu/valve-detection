#include "valve_detection/valve_detector.hpp"

namespace valve_detection {

ValveDetector::ValveDetector(int yolo_img_width_,
                             int yolo_img_height_,
                             float annulus_radius_ratio,
                             float plane_ransac_threshold,
                             int plane_ransac_max_iterations,
                             float valve_handle_offset)
    : yolo_img_width_(yolo_img_width_),
      yolo_img_height_(yolo_img_height_),
      annulus_radius_ratio_(annulus_radius_ratio),
      plane_ransac_threshold_(plane_ransac_threshold),
      plane_ransac_max_iterations_(plane_ransac_max_iterations),
      valve_handle_offset_(valve_handle_offset) {}

void ValveDetector::init_angle_detector(const AngleDetectorParams& params) {
    angle_detector_ = std::make_unique<AngleDetector>(params);
}

cv::Mat ValveDetector::draw_detections(const cv::Mat& image,
                                       const std::vector<BoundingBox>& boxes,
                                       const std::vector<Pose>& poses) const {
    if (boxes.size() != poses.size()) {
        return cv::Mat();
    }
    std::vector<BoundingBox> transformed_boxes;
    transformed_boxes.reserve(boxes.size());
    for (const auto& box : boxes) {
        transformed_boxes.push_back(transform_bounding_box(box));
    }

    cv::Mat visualized_image = image.clone();

    cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << color_image_properties_.intr.fx, 0,
         color_image_properties_.intr.cx, 0, color_image_properties_.intr.fy,
         color_image_properties_.intr.cy, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);

    for (size_t i = 0; i < transformed_boxes.size(); ++i) {
        const auto& box = transformed_boxes[i];
        const auto& pose = poses[i];

        int x1 = box.center_x - box.size_x / 2;
        int y1 = box.center_y - box.size_y / 2;
        int x2 = box.center_x + box.size_x / 2;
        int y2 = box.center_y + box.size_y / 2;
        cv::rectangle(visualized_image, cv::Point(x1, y1), cv::Point(x2, y2),
                      cv::Scalar(0, 255, 0), 2);

        cv::Mat tvec = (cv::Mat_<double>(3, 1) << pose.position[0],
                        pose.position[1], pose.position[2]);

        Eigen::Matrix3f rotation_matrix = pose.orientation.toRotationMatrix();
        cv::Mat rmat = (cv::Mat_<double>(3, 3) << rotation_matrix(0, 0),
                        rotation_matrix(0, 1), rotation_matrix(0, 2),
                        rotation_matrix(1, 0), rotation_matrix(1, 1),
                        rotation_matrix(1, 2), rotation_matrix(2, 0),
                        rotation_matrix(2, 1), rotation_matrix(2, 2));
        cv::Mat rvec;
        cv::Rodrigues(rmat, rvec);

        cv::drawFrameAxes(visualized_image, camera_matrix, dist_coeffs, rvec,
                          tvec, 0.1);
    }
    return visualized_image;
}

void ValveDetector::rmat_to_quat(const Eigen::Matrix3f& rotation_matrix,
                                 Eigen::Quaternionf& quat) const {
    quat = Eigen::Quaternionf(rotation_matrix);
    quat.normalize();
}

void ValveDetector::project_point_to_pixel(float x,
                                           float y,
                                           float z,
                                           int& u,
                                           int& v) const {
    if (z <= 0) {
        u = v = -1;  // Invalid pixel coordinates
        return;
    }

    double fx = color_image_properties_.intr.fx;
    double fy = color_image_properties_.intr.fy;
    double cx = color_image_properties_.intr.cx;
    double cy = color_image_properties_.intr.cy;

    u = static_cast<int>((x * fx / z) + cx);
    v = static_cast<int>((y * fy / z) + cy);
}

void ValveDetector::calculate_letterbox_padding() {
    int org_image_width = color_image_properties_.dim.width;
    int org_image_height = color_image_properties_.dim.height;

    letterbox_scale_factor_ =
        std::min(static_cast<double>(yolo_img_width_) / org_image_width,
                 static_cast<double>(yolo_img_height_) / org_image_height);

    double resized_width = org_image_width * letterbox_scale_factor_;
    double resized_height = org_image_height * letterbox_scale_factor_;

    letterbox_pad_x_ = (yolo_img_width_ - resized_width) / 2.0;
    letterbox_pad_y_ = (yolo_img_height_ - resized_height) / 2.0;
}

BoundingBox ValveDetector::transform_bounding_box(
    const BoundingBox& bbox) const {
    BoundingBox transformed_bbox = bbox;

    transformed_bbox.center_x =
        (bbox.center_x - letterbox_pad_x_) / letterbox_scale_factor_;
    transformed_bbox.center_y =
        (bbox.center_y - letterbox_pad_y_) / letterbox_scale_factor_;
    transformed_bbox.size_x /= letterbox_scale_factor_;
    transformed_bbox.size_y /= letterbox_scale_factor_;

    return transformed_bbox;
}

bool ValveDetector::segment_plane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
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

Eigen::Vector3f ValveDetector::get_ray_direction(
    const BoundingBox& bbox) const {
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

Eigen::Vector3f ValveDetector::compute_plane_normal(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction) const {
    Eigen::Vector3f normal_vector;

    if (!coefficients || coefficients->values.size() < 3) {
        return Eigen::Vector3f::Zero();
    }

    normal_vector << coefficients->values[0], coefficients->values[1],
        coefficients->values[2];

    normal_vector.normalize();

    if (normal_vector.dot(ray_direction) > 0.0) {
        normal_vector = -normal_vector;
    }

    return normal_vector;
}

Eigen::Vector3f ValveDetector::find_ray_plane_intersection(
    const pcl::ModelCoefficients::Ptr& coefficients,
    const Eigen::Vector3f& ray_direction) const {
    if (!coefficients || coefficients->values.size() < 4) {
        return Eigen::Vector3f::Zero();
    }

    Eigen::Vector3f plane_normal;
    plane_normal << coefficients->values[0], coefficients->values[1],
        coefficients->values[2];
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

Eigen::Vector3f ValveDetector::shift_point_along_normal(
    const Eigen::Vector3f& intersection_point,
    const Eigen::Vector3f& plane_normal) const {
    return intersection_point + (plane_normal * valve_handle_offset_);
}

Eigen::Matrix3f ValveDetector::create_rotation_matrix(
    const Eigen::Vector3f& plane_normal,
    float angle) {
    Eigen::Vector3f z_axis = plane_normal;

    Eigen::Vector3f temp_vec(1, 0, 0);
    if (std::abs(z_axis.dot(temp_vec)) > 0.99f) {
        temp_vec = Eigen::Vector3f(0, 1, 0);
    }
    Eigen::Vector3f base_x_axis = z_axis.cross(temp_vec).normalized();

    Eigen::AngleAxisf rotation(angle, z_axis);
    Eigen::Vector3f x_axis = rotation * base_x_axis;

    if (filter_direction_.dot(x_axis) < 0) {
        x_axis = -x_axis;
    }
    filter_direction_ = x_axis;

    Eigen::Vector3f y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;

    return rotation_matrix;
}

}  // namespace valve_detection
