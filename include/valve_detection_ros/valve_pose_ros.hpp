#ifndef VALVE_POSE_DEPTH_ROS_HPP
#define VALVE_POSE_DEPTH_ROS_HPP

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "valve_detection/valve_detector.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace valve_detection {

/**
 * @class ValvePoseNode
 * @brief ROS 2 node for computing valve poses from depth images or point clouds
 * and 2D detections.
 *
 * This node subscribes to depth images or point clouds, color images
 * (optional), and 2D detection messages. It synchronizes the inputs, computes
 * valve poses using ValveDetector, and publishes results as PoseArray and
 * optionally as visualized point clouds or images.
 */
class ValvePoseNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructs the ValvePoseNode with ROS node options.
     * @param options Node options.
     */
    explicit ValvePoseNode(const rclcpp::NodeOptions& options);

    virtual ~ValvePoseNode() = default;

   private:
    /**
     * @brief Callback for camera info subscription to obtain intrinsics and
     * image size.
     * @param camera_info_msg Shared pointer to the CameraInfo message.
     */
    void color_image_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    /**
     * @brief Initializes the ValveDetector with the specified ros parameters.
     */
    void init_valve_detector();

    /**
     * @brief Initializes the angle detector inside the ValveDetector if angle
     * calculation is enabled.
     */
    void init_angle_detector();

    /**
     * @brief Converts a vision_msgs::BoundingBox2D to local BoundingBox type.
     * @param bbox The input 2D bounding box message.
     * @return Converted BoundingBox.
     */
    BoundingBox to_bounding_box(
        const vision_msgs::msg::BoundingBox2D& bbox) const;

    /**
     * @brief Publishes a point cloud containing annulus points.
     * @param cloud The point cloud to publish.
     * @param header ROS message header to use for publication.
     */
    void publish_annulus_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                             const std_msgs::msg::Header& header) const;

    /**
     * @brief Publishes a point cloud containing segmented plane points.
     * @param cloud The point cloud to publish.
     * @param header ROS message header to use for publication.
     */
    void publish_annulus_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                               const std_msgs::msg::Header& header) const;

    /**
     * @brief Publishes computed valve poses as a PoseArray message.
     * @param poses Vector of computed valve poses.
     * @param header ROS message header to use for publication.
     */
    void publish_valve_poses(const std::vector<Pose>& poses,
                             const std_msgs::msg::Header& header) const;

    /**
     * @brief Configures message_filters synchronizers based on available
     * inputs.
     */
    void setup_sync();

    /**
     * @brief Callback for Depth Image + Color Image + Detections
     * synchronization.
     */
    void di_ci_d_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    /**
     * @brief Callback for Depth Image + Detections synchronization (no color
     * image).
     */
    void di_d_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    /**
     * @brief Callback for Point Cloud + Color Image + Detections
     * synchronization.
     */
    void pc_ci_d_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    /**
     * @brief Callback for Point Cloud + Detections synchronization (no color
     * image).
     */
    void pc_d_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);

    /**
     * @brief Generic template callback for synchronized inputs.
     *
     * Supports T = sensor_msgs::msg::Image (depth image) or PointCloud2.
     * Handles optional color image and converts messages to OpenCV/PCL formats.
     *
     * @tparam T Depth input message type.
     * @param depth_msg Depth image or point cloud message.
     * @param color_image_msg Optional color image message.
     * @param detections 2D detection message.
     */
    template <typename T>
    void synchronized_callback(
        const std::shared_ptr<const T>& depth_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image_msg,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
        if (!color_image_info_received_) {
            return;
        }
        if (detections->detections.empty() && !visualize_detections_) {
            return;
        }

        cv::Mat cv_depth_image;
        sensor_msgs::msg::PointCloud2 pcl_tf;
        cv::Mat cv_color_image;

        if constexpr (std::is_same<T, sensor_msgs::msg::PointCloud2>::value) {
            std::string source_frame = depth_msg->header.frame_id;
            std::string target_frame = color_image_frame_id_;

            try {
                geometry_msgs::msg::TransformStamped transform =
                    tf_buffer_->lookupTransform(
                        target_frame, source_frame,
                        rclcpp::Time(depth_msg->header.stamp));

                tf2::doTransform(*depth_msg, pcl_tf, transform);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to transform point cloud: %s", ex.what());
                return;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(pcl_tf, *cloud);

        } else if constexpr (std::is_same<T, sensor_msgs::msg::Image>::value) {
            cv_bridge::CvImageConstPtr cv_depth_ptr =
                cv_bridge::toCvShare(depth_msg, "32FC1");
            cv_depth_image = cv_depth_ptr->image;
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Unsupported message type in synchronized_callback.");
            return;
        }

        if (use_color_image_) {
            if (!color_image_msg) {
                RCLCPP_ERROR(this->get_logger(),
                             "Color image message is null.");
                return;
            }
            cv_bridge::CvImageConstPtr cv_ptr =
                cv_bridge::toCvShare(color_image_msg, "bgr8");

            cv_color_image = cv_ptr->image;
        }

        std::vector<BoundingBox> boxes;
        boxes.reserve(detections->detections.size());
        for (const auto& detection : detections->detections) {
            boxes.push_back(to_bounding_box(detection.bbox));
        }

        std::vector<Pose> poses;
        poses.reserve(boxes.size());

        pcl::PointCloud<pcl::PointXYZ>::Ptr all_annulus_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_annulus_plane_cloud;

        if constexpr (std::is_same<T, sensor_msgs::msg::PointCloud2>::value) {
            valve_detector_->Compute_valve_poses(
                pcl_tf, cv_color_image, boxes, poses, all_annulus_cloud,
                all_annulus_plane_cloud, pcl_visualize_, calculate_angle_);
        } else if constexpr (std::is_same<T, sensor_msgs::msg::Image>::value) {
            valve_detector_->Compute_valve_poses(
                cv_depth_image, cv_color_image, boxes, poses, all_annulus_cloud,
                all_annulus_plane_cloud, pcl_visualize_, calculate_angle_);
        }
        if (pcl_visualize_ && all_annulus_cloud && all_annulus_plane_cloud) {
            publish_annulus_pcl(all_annulus_cloud, depth_msg->header);
            publish_annulus_plane(all_annulus_plane_cloud, depth_msg->header);
        }
        if (visualize_detections_) {
            cv::Mat visualized_image =
                valve_detector_->draw_detections(cv_color_image, boxes, poses);
            sensor_msgs::msg::Image::SharedPtr output_msg =
                cv_bridge::CvImage(depth_msg->header, "bgr8", visualized_image)
                    .toImageMsg();
            processed_image_pub_->publish(*output_msg);
        }
        publish_valve_poses(poses, depth_msg->header);
    }

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        color_image_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray>
        detections_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
        valve_poses_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        annulus_pcl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        annulus_plane_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub_;

    // Depth image sync policies uses ExactTime with the assumption
    // that the depth image and color image are from the same camera
    // and are synchronized.

    // Policy for Depth Image + Color Image + Detections
    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>
        SyncPolicy_DI_CI_D;

    // Policy for Depth Image + Detections (no color image)
    typedef message_filters::sync_policies::
        ExactTime<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>
            SyncPolicy_DI_D;

    // Policy for Point Cloud + Color Image + Detections
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>
        SyncPolicy_PC_CI_D;

    // Policy for Point Cloud + Detections (no color image)
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        vision_msgs::msg::Detection2DArray>
        SyncPolicy_PC_D;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_DI_CI_D>>
        sync_di_ci_d_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_DI_D>> sync_di_d_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_PC_CI_D>>
        sync_pc_ci_d_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy_PC_D>> sync_pc_d_;

    bool color_image_info_received_ = false;
    std::unique_ptr<ValveDetector> valve_detector_;
    std::string color_image_frame_id_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool visualize_detections_ = true;
    bool pcl_visualize_ = false;
    bool calculate_angle_ = true;
    bool use_color_image_ = true;
    bool use_depth_image_ = true;
};

}  // namespace valve_detection

#endif  // VALVE_POSE_DEPTH_ROS_HPP
