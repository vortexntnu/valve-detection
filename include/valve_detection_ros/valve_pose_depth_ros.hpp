#ifndef VALVE_POSE_DEPTH_ROS_HPP
#define VALVE_POSE_DEPTH_ROS_HPP

#include "valve_detection/valve_pose_depth.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>


namespace valve_detection {

class ValvePoseDepthNode {
public:
    explicit ValvePoseDepthNode(const rclcpp::NodeOptions& options);
    virtual ~ValvePoseDepthNode() = default;

    
private:
    void color_image_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

    BoundingBox to_bounding_box(const vision_msgs::msg::BoundingBox2D& bbox) const;

    void publish_annulus_pcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const;

    /**
     * @brief Callback function for synchronized depth image, color image, and
     * 2D detections.
     *
     * This function is triggered when synchronized messages for a depth image,
     * a color image, and a 2D detection array are received.
     */
     void synchronized_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
        const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections);


    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr color_image_info_sub_;
    bool color_image_info_received_ = false;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_image_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray>
        detections_sub_;

    typedef message_filters::sync_policies::ExactTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        vision_msgs::msg::Detection2DArray>
        MySyncPolicy;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        valve_pose_pub_;

    std::unique_ptr<ValvePoseDepth> valve_detector_;

    std::string color_image_frame_id_;

    bool visualize_detections_ = true;
    bool debug_visualize_ = false;
    bool calculate_angle_ = true;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annulus_pcl_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr plane_normal_pub_;
}

} // namespace valve_detection

#endif // VALVE_POSE_DEPTH_ROS_HPP