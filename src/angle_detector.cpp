#include "valve_detection/angle_detector.hpp"
#include <limits>
#include "valve_detection/types.hpp"

namespace valve_detection {

void AngleDetector::compute_angles(const cv::Mat& color_image,
                                   std::vector<BoundingBox>& boxes) const {
    for (auto& box : boxes) {
        double width = box.size_x;
        double height = box.size_y;
        double center_x = box.center_x;
        double center_y = box.center_y;

        int x1 = std::max(
            static_cast<int>(center_x - width * params_.line_detection_area),
            0);
        int y1 = std::max(
            static_cast<int>(center_y - height * params_.line_detection_area),
            0);
        int x2 = std::min(
            static_cast<int>(center_x + width * params_.line_detection_area),
            color_image.cols - 1);
        int y2 = std::min(
            static_cast<int>(center_y + height * params_.line_detection_area),
            color_image.rows - 1);

        if (x1 >= x2 || y1 >= y2) {
            box.theta = std::numeric_limits<double>::quiet_NaN();
            continue;
        }

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
                        params_.hough_min_line_length,
                        params_.hough_max_line_gap);

        if (lines.empty()) {
            box.theta = std::numeric_limits<double>::quiet_NaN();
            continue;
        }

        cv::Vec4i longest_line = find_longest_line(lines);

        if (longest_line[0] >= longest_line[2]) {
            int temp_x = longest_line[0];
            int temp_y = longest_line[1];
            longest_line[0] = longest_line[2];
            longest_line[1] = longest_line[3];
        }

        float theta = std::atan2(longest_line[3] - longest_line[1],
                                 longest_line[2] - longest_line[0]);

        if (theta < 0) {
            theta += CV_PI;
        }
        if (theta > CV_PI) {
            theta -= CV_PI;
        }
        box.theta = theta;
    }
}

void AngleDetector::compute_angles_debug(
    cv::Mat& image_to_draw_on,
    std::vector<BoundingBox>& boxes) const {
    const cv::Scalar ALL_LINES_COLOR(255, 0, 0);
    const cv::Scalar LONGEST_LINE_COLOR(0, 255, 0);
    const int LINE_THICKNESS = 2;

    cv::Mat output_image = cv::Mat::zeros(image_to_draw_on.size(), CV_8UC3);

    for (auto& box : boxes) {
        double width = box.size_x;
        double height = box.size_y;
        double center_x = box.center_x;
        double center_y = box.center_y;

        int x1 = std::max(
            static_cast<int>(center_x - width * params_.line_detection_area),
            0);
        int y1 = std::max(
            static_cast<int>(center_y - height * params_.line_detection_area),
            0);
        int x2 = std::min(
            static_cast<int>(center_x + width * params_.line_detection_area),
            image_to_draw_on.cols - 1);
        int y2 = std::min(
            static_cast<int>(center_y + height * params_.line_detection_area),
            image_to_draw_on.rows - 1);

        if (x1 >= x2 || y1 >= y2) {
            box.theta = std::numeric_limits<double>::quiet_NaN();
            continue;
        }

        cv::Mat roi = image_to_draw_on(cv::Rect(x1, y1, x2 - x1, y2 - y1));

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
                        params_.hough_min_line_length,
                        params_.hough_max_line_gap);

        if (lines.empty()) {
            box.theta = std::numeric_limits<double>::quiet_NaN();

            cv::Mat output_roi_gray =
                output_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));
            cv::Mat edges_color;
            cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
            edges_color.copyTo(output_roi_gray, mask);

            continue;
        }

        cv::Mat output_roi = output_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        cv::Mat edges_color;
        cv::cvtColor(edges, edges_color, cv::COLOR_GRAY2BGR);
        edges_color.copyTo(output_roi, mask);

        const cv::Point offset(x1, y1);
        for (const auto& line : lines) {
            cv::line(output_image, cv::Point(line[0], line[1]) + offset,
                     cv::Point(line[2], line[3]) + offset, ALL_LINES_COLOR,
                     LINE_THICKNESS);
        }

        cv::Vec4i longest_line = find_longest_line(lines);
        cv::line(output_image,
                 cv::Point(longest_line[0], longest_line[1]) + offset,
                 cv::Point(longest_line[2], longest_line[3]) + offset,
                 LONGEST_LINE_COLOR, LINE_THICKNESS);

        box.theta = std::atan2(longest_line[3] - longest_line[1],
                               longest_line[2] - longest_line[0]);
    }

    image_to_draw_on = output_image;
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
