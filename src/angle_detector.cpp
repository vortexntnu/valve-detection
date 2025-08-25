#include "valve_detection/angle_detector.hpp"
#include <limits>
#include "valve_detection/types.hpp"

namespace valve_detection {

double AngleDetector::calculate_angle(const cv::Mat& color_image,
                                      const BoundingBox& bbox) const {
    double width = bbox.size_x;
    double height = bbox.size_y;
    double center_x = bbox.center_x;
    double center_y = bbox.center_y;

    int x1 = std::max(
        static_cast<int>(center_x - width * params_.line_detection_area), 0);
    int y1 = std::max(
        static_cast<int>(center_y - height * params_.line_detection_area), 0);
    int x2 = std::min(
        static_cast<int>(center_x + width * params_.line_detection_area),
        color_image.cols - 1);
    int y2 = std::min(
        static_cast<int>(center_y + height * params_.line_detection_area),
        color_image.rows - 1);

    cv::Mat roi = color_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));

    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::Point center((roi.cols / 2), (roi.rows / 2));
    cv::Size axes(roi.cols / 2, roi.rows / 2);
    cv::ellipse(mask, center, axes, 0, 0, 360, cv::Scalar(255), -1);

    cv::Mat masked_roi;
    roi.copyTo(masked_roi, mask);

    cv::Mat gray;
    cv::cvtColor(masked_roi, gray, cv::COLOR_BGR2GRAY);

    cv::Mat edges = apply_canny_edge_detection(gray);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, params_.hough_rho_res,
                    params_.hough_theta_res, params_.hough_threshold,
                    params_.hough_min_line_length, params_.hough_max_line_gap);

    if (lines.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    cv::Vec4i longest_line = find_longest_line(lines);
    return std::atan2(longest_line[3] - longest_line[1],
                      longest_line[2] - longest_line[0]);
}

cv::Vec4i AngleDetector::find_longest_line(
    const std::vector<cv::Vec4i>& lines) const {
    if (lines.empty()) {
        return cv::Vec4i(0, 0, 0, 0);
    }

    cv::Vec4i longest_line = lines[0];
    double max_length = 0;

    for (const auto& line : lines) {
        double length = std::hypot(line[2] - line[0], line[3] - line[1]);
        if (length > max_length) {
            max_length = length;
            longest_line = line;
        }
    }
    return longest_line;
}

cv::Mat AngleDetector::apply_canny_edge_detection(
    const cv::Mat& gray_image) const {
    cv::Mat edges;
    cv::Canny(gray_image, edges, params_.canny_low_threshold,
              params_.canny_high_threshold, params_.canny_aperture_size);
    return edges;
}

}  // namespace valve_detection
