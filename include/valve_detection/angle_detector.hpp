#ifndef VALVE_POSE_ANGLE_HPP
#define VALVE_POSE_ANGLE_HPP

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>
#include "valve_detection/types.hpp"

namespace valve_detection {

class AngleDetector {
   public:
    /**
     * @brief Constructs an AngleDetector with a set of parameters.
     * @param params The parameters for the detector.
     */
    explicit AngleDetector(const AngleDetectorParams& params)
        : params_(params) {}

    /**
     * @brief Calculates the angle of the longest line within a detected valve.
     * @param color_image The input color image.
     * @param bbox The bounding box of the detected valve.
     * @return The angle in radians, or NaN if no lines are detected.
     */
    double calculate_angle(const cv::Mat& color_image,
                           const BoundingBox& bbox) const;

   private:
    /**
     * @brief Finds the longest line among a set of lines detected in an image.
     * @param lines A vector of lines, each represented as cv::Vec4i (x1, y1,
     * x2, y2).
     * @return The cv::Vec4i representing the longest line.
     * If the input vector is empty, returns a zero-length line {0, 0, 0, 0}.
     */
    cv::Vec4i find_longest_line(const std::vector<cv::Vec4i>& lines) const;

    /**
     * @brief Applies Canny edge detection to a grayscale image.
     * @param gray_image The input grayscale image.
     * @return A binary image (CV_8UC1) with edges detected.
     */
    cv::Mat apply_canny_edge_detection(const cv::Mat& gray_image) const;

    /** @brief Parameters for the angle detector. */
    AngleDetectorParams params_;
};

}  // namespace valve_detection

#endif  // VALVE_POSE_ANGLE_HPP
