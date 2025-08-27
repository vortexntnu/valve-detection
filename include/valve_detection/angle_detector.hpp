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
     * @brief Computes the angle for each bounding box in a vector.
     *
     * This function processes each bounding box to define a region of interest
     * (ROI) in the input image, applies Canny edge detection and Hough line
     * transform, and then calculates the dominant line angle within each box's
     * ROI, and updates the 'theta' member of each box in-place.
     *
     * @param color_image The input image to be processed.
     * @param boxes A vector of BoundingBox objects passed by reference. The
     * 'theta' member of each box will be updated.
     */
    void compute_angles(const cv::Mat& color_image,
                        std::vector<BoundingBox>& boxes) const;

    /**
     * @brief Processes a vector of bounding boxes to calculate the angle for
     * each, updating the boxes in-place and drawing all visualizations on a
     * single image.
     *
     * @param image_to_draw_on The input/output image. This cv::Mat will be
     * modified to include visualizations for all processed boxes.
     * @param boxes A vector of BoundingBox objects. The theta member of each
     * box will be updated with the calculated angle (or NaN if no line is
     * found).
     */
    void compute_angles_debug(cv::Mat& image_to_draw_on,
                              std::vector<BoundingBox>& boxes) const;

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
