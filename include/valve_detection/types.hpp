#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Dense>

namespace valve_detection {

/**
 * @struct CameraIntrinsics
 * @brief Stores intrinsic parameters of a pinhole camera model.
 */
struct CameraIntrinsics {
    double fx;  ///< Focal length in pixels along x-axis
    double fy;  ///< Focal length in pixels along y-axis
    double cx;  ///< Principal point x-coordinate (in pixels)
    double cy;  ///< Principal point y-coordinate (in pixels)
};

/**
 * @struct ImageDimensions
 * @brief Represents the width and height of an image.
 */
struct ImageDimensions {
    int width;   ///< Image width in pixels
    int height;  ///< Image height in pixels
};

/**
 * @struct ImageProperties
 * @brief Combines camera intrinsics and image dimensions.
 */
struct ImageProperties {
    CameraIntrinsics intr;  ///< Camera intrinsic parameters
    ImageDimensions dim;    ///< Image width and height
};

/**
 * @struct BoundingBox
 * @brief Represents a 2D bounding box with center, size, and orientation.
 */
struct BoundingBox {
    float center_x;  ///< X-coordinate of the bounding box center
    float center_y;  ///< Y-coordinate of the bounding box center
    float size_x;    ///< Width of the bounding box
    float size_y;    ///< Height of the bounding box
    float theta;     ///< Orientation in radians (positive counter-clockwise)
};

/**
 * @struct Pose
 * @brief Represents a 3D pose with position and orientation.
 */
struct Pose {
    Eigen::Vector3f position;        ///< 3D position (x, y, z)
    Eigen::Quaternionf orientation;  ///< Orientation as a quaternion
};

/**
 * @struct AngleDetectorParams
 * @brief Parameters for Canny edge detection and Hough line transform used in
 * angle detection.
 */
struct AngleDetectorParams {
    float line_detection_area;  ///< Fraction of the bounding box to use as ROI
                                ///< (0.0–1.0)

    // Canny edge detection parameters
    int canny_low_threshold;   ///< Low threshold for Canny edge detector
    int canny_high_threshold;  ///< High threshold for Canny edge detector
    int canny_aperture_size;  ///< Aperture size for the Sobel operator in Canny

    // Hough line transform parameters
    double hough_rho_res;    ///< Distance resolution in pixels
    double hough_theta_res;  ///< Angle resolution in radians
    int hough_threshold;  ///< Minimum number of votes (intersections) to detect
                          ///< a line
    double
        hough_min_line_length;  ///< Minimum line length to accept (in pixels)
    double hough_max_line_gap;  ///< Maximum allowed gap between line segments
                                ///< to treat them as a single line
};

}  // namespace valve_detection

#endif  // TYPES_HPP
