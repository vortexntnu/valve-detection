#ifndef VALVE_POSE_BASE_HPP
#define VALVE_POSE_BASE_HPP

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
#include "valve_detection/depth_image_processing.hpp"
#include "valve_detection/pointcloud_processing.hpp"
#include "valve_detection/types.hpp"

namespace valve_detection {

/**
 * @class ValveDetector
 * @brief Detects valve poses from color and depth images or point clouds.
 *
 * This class extracts valve annulus point clouds, segments planes,
 * computes ray-plane intersections, estimates valve poses, and can calculate
 * the valve rotation angle using the AngleDetector.
 */
class ValveDetector {
   public:
    /**
     * @brief Constructs a ValveDetector with YOLO image dimensions and
     * detection parameters.
     * @param yolo_img_width Width of the YOLO input image.
     * @param yolo_img_height Height of the YOLO input image.
     * @param annulus_radius_ratio Ratio of bounding box used to define the
     * annulus.
     * @param plane_ransac_threshold Distance threshold for plane segmentation.
     * @param plane_ransac_max_iterations Max iterations for RANSAC plane
     * segmentation.
     * @param valve_handle_offset Distance to shift the valve handle along the
     * plane normal.
     */
    ValveDetector(int yolo_img_width,
                  int yolo_img_height_,
                  float annulus_radius_ratio,
                  float plane_ransac_threshold,
                  int plane_ransac_max_iterations,
                  float valve_handle_offset);

    virtual ~ValveDetector() = default;

    /**
     * @brief Computes valve poses from depth input and bounding boxes.
     *
     * This template function supports depth input as either a PCL point cloud
     * or an OpenCV depth image. It extracts the annulus points, segments
     * planes, computes ray-plane intersections, shifts points along normals,
     * and optionally calculates valve rotation angles.
     *
     * @tparam T Depth input type (cv::Mat or
     * pcl::PointCloud<pcl::PointXYZ>::Ptr).
     * @param depth_input The input depth data.
     * @param color_image Corresponding color image for angle calculation.
     * @param boxes Bounding boxes of detected valves.
     * @param poses Output vector of computed valve poses.
     * @param all_annulus_cloud Optional point cloud for visualizing all annuli.
     * @param all_annulus_plane_cloud Optional point cloud for visualizing
     * segmented planes.
     * @param debug_visualize Whether to populate visualization point clouds.
     * @param calculate_angle Whether to compute the valve rotation angle.
     */
    template <typename T>
    void Compute_valve_poses(
        const T& depth_input,
        const std::vector<BoundingBox>& boxes,
        std::vector<Pose>& poses,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& all_annulus_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& all_annulus_plane_cloud,
        bool debug_visualize) {
        if (debug_visualize) {
            all_annulus_cloud->clear();
            all_annulus_plane_cloud->clear();
        }

        for (const auto& box : boxes) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr annulus_cloud(
                new pcl::PointCloud<pcl::PointXYZ>);

            if constexpr (std::is_same<
                              T, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value) {
                extract_annulus_pcl(depth_input, box, color_image_properties_,
                                    annulus_radius_ratio_, annulus_cloud);
            } else if constexpr (std::is_same<T, cv::Mat>::value) {
                extract_annulus_pcl(depth_input, box, color_image_properties_,
                                    annulus_radius_ratio_, annulus_cloud);
            }

            if (annulus_cloud->empty())
                continue;

            if (debug_visualize) {
                *all_annulus_cloud += *annulus_cloud;
            }

            pcl::ModelCoefficients::Ptr coefficients(
                new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

            if (!segment_plane(annulus_cloud, coefficients, inliers))
                continue;

            if (debug_visualize) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr annulus_plane_cloud(
                    new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*annulus_cloud, inliers->indices,
                                    *annulus_plane_cloud);
                *all_annulus_plane_cloud += *annulus_plane_cloud;
            }

            Eigen::Vector3f bb_centre_ray = get_ray_direction(box);
            Eigen::Vector3f ray_plane_intersection =
                find_ray_plane_intersection(coefficients, bb_centre_ray);
            if (ray_plane_intersection.isZero())
                continue;

            Eigen::Vector3f plane_normal =
                compute_plane_normal(coefficients, bb_centre_ray);
            if (plane_normal.isZero())
                continue;

            Eigen::Vector3f shifted_position =
                shift_point_along_normal(ray_plane_intersection, plane_normal);

            float angle = box.theta;

            Eigen::Matrix3f rotation_matrix =
                create_rotation_matrix(plane_normal, angle);
            Eigen::Quaternionf rotation_quat;
            rmat_to_quat(rotation_matrix, rotation_quat);

            Pose pose;
            pose.position = shifted_position;
            pose.orientation = rotation_quat;
            poses.emplace_back(pose);
        }
    }

    /**
     * @brief Projects a 3D point to pixel coordinates using the camera
     * intrinsics.
     * @param x X coordinate in 3D space.
     * @param y Y coordinate in 3D space.
     * @param z Z coordinate in 3D space.
     * @param[out] u Output pixel x-coordinate.
     * @param[out] v Output pixel y-coordinate.
     */
    void project_point_to_pixel(float x,
                                float y,
                                float z,
                                int& u,
                                int& v) const;

    /**
     * @brief Calculates the padding and scaling for letterbox resizing.
     */
    void calculate_letterbox_padding();

    /**
     * @brief Transforms bounding box coordinates from letterboxed YOLO input to
     * original image coordinates.
     * @param bbox Bounding box in YOLO image coordinates.
     * @return Bounding box transformed to original image coordinates.
     */
    BoundingBox transform_bounding_box(const BoundingBox& bbox) const;

    /**
     * @brief Sets the color image properties (intrinsics and dimensions).
     */
    void set_color_image_properties(const ImageProperties& properties) {
        color_image_properties_ = properties;
    }

    /**
     * @brief Segments the dominant plane from a point cloud using RANSAC.
     * @param cloud Input point cloud.
     * @param coefficients Output model coefficients (Ax + By + Cz + D = 0).
     * @param inliers Output inlier indices of the plane.
     * @return True if a plane with inliers was found, false otherwise.
     */
    bool segment_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       pcl::ModelCoefficients::Ptr& coefficients,
                       pcl::PointIndices::Ptr& inliers) const;

    /**
     * @brief Computes the ray direction vector through the bounding box center
     * in camera frame.
     * @param bbox Bounding box for the valve.
     * @return Normalized Eigen::Vector3f representing the ray direction.
     */
    Eigen::Vector3f get_ray_direction(const BoundingBox& bbox) const;

    /**
     * @brief Computes the plane normal from plane coefficients, ensuring it
     * points against the ray.
     * @param coefficients Plane coefficients (Ax + By + Cz + D = 0).
     * @param ray_direction Ray direction vector from camera.
     * @return Normalized Eigen::Vector3f plane normal, or zero if invalid.
     */
    Eigen::Vector3f compute_plane_normal(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& ray_direction) const;

    /**
     * @brief Finds the intersection point of a ray and a plane.
     * @param coefficients Plane coefficients.
     * @param ray_direction Normalized ray direction.
     * @return Intersection point in 3D space, or zero vector if no
     * intersection.
     */
    Eigen::Vector3f find_ray_plane_intersection(
        const pcl::ModelCoefficients::Ptr& coefficients,
        const Eigen::Vector3f& ray_direction) const;

    /**
     * @brief Shifts a point along a plane normal by the valve handle offset.
     * @param intersection_point Point on the plane.
     * @param plane_normal Normalized plane normal vector.
     * @return Shifted 3D point representing valve handle position.
     */
    Eigen::Vector3f shift_point_along_normal(
        const Eigen::Vector3f& intersection_point,
        const Eigen::Vector3f& plane_normal) const;

    /**
     * @brief Creates a rotation matrix with the plane normal as z-axis and
     * valve angle applied.
     * @param plane_normal Normalized plane normal vector.
     * @param angle Valve rotation angle in radians.
     * @return Rotation matrix (Eigen::Matrix3f) for the valve pose.
     */
    Eigen::Matrix3f create_rotation_matrix(const Eigen::Vector3f& plane_normal,
                                           float angle);

    /**
     * @brief Converts a 3x3 rotation matrix to a normalized quaternion.
     * @param rotation_matrix Input rotation matrix.
     * @param quat Output quaternion.
     */
    void rmat_to_quat(const Eigen::Matrix3f& rotation_matrix,
                      Eigen::Quaternionf& quat) const;

    /**
     * @brief Draws bounding boxes and 3D axes for valve poses on an image.
     * @param image Input color image.
     * @param boxes Detected valve bounding boxes.
     * @param poses Computed valve poses.
     * @return Image with drawn detections.
     */
    cv::Mat draw_detections(const cv::Mat& image,
                            const std::vector<BoundingBox>& boxes,
                            const std::vector<Pose>& poses) const;

   protected:
    ImageProperties color_image_properties_;
    int yolo_img_width_;
    int yolo_img_height_;
    float annulus_radius_ratio_;
    float plane_ransac_threshold_;
    int plane_ransac_max_iterations_;
    float valve_handle_offset_;
    float letterbox_scale_factor_;
    int letterbox_pad_x_;
    int letterbox_pad_y_;
    Eigen::Vector3f filter_direction_ = Eigen::Vector3f(1, 0, 0);
};

}  // namespace valve_detection

#endif  // VALVE_POSE_BASE_HPP
