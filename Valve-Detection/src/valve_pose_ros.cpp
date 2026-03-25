// valve_pose_ros.cpp
// ROS node: subscribes to depth and detections; publishes valve poses
// and visualizations.
#include "valve_detection_ros/valve_pose_ros.hpp"
#include "valve_detection_ros/ros_utils.hpp"

#include <opencv2/imgproc.hpp>

#include <cmath>

namespace valve_detection {

using std::placeholders::_1;
using std::placeholders::_2;

ValvePoseNode::ValvePoseNode(const rclcpp::NodeOptions& options)
    : Node("valve_pose_node", options) {
    debug_visualize_ = declare_parameter<bool>("debug_visualize");
    iou_duplicate_threshold_ = static_cast<float>(
        declare_parameter<double>("iou_duplicate_threshold"));
    const std::string drone = declare_parameter<std::string>("drone", "moby");
    const std::string frame_base =
        declare_parameter<std::string>("output_frame_id");
    output_frame_id_ = frame_base.empty() ? "" : drone + "/" + frame_base;
    landmark_type_ = declare_parameter<int>("landmark_type");

    setup_estimator();

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    setup_publishers(qos);
    setup_subscribers(qos);
}

void ValvePoseNode::setup_estimator() {
    const int yolo_w = declare_parameter<int>("yolo_img_width");
    const int yolo_h = declare_parameter<int>("yolo_img_height");
    const float annulus_ratio =
        declare_parameter<float>("annulus_radius_ratio");
    const float ransac_thresh =
        declare_parameter<float>("plane_ransac_threshold");
    const int ransac_iters =
        declare_parameter<int>("plane_ransac_max_iterations");
    const float handle_off = declare_parameter<float>("valve_handle_offset");

    detector_ = std::make_unique<PoseEstimator>(
        yolo_w, yolo_h, annulus_ratio, ransac_thresh, ransac_iters, handle_off);

    depth_color_extrinsic_.R = Eigen::Matrix3f::Identity();
    depth_color_extrinsic_.t.x() =
        static_cast<float>(declare_parameter<double>("depth_to_color_tx"));
    depth_color_extrinsic_.t.y() =
        static_cast<float>(declare_parameter<double>("depth_to_color_ty"));
    depth_color_extrinsic_.t.z() =
        static_cast<float>(declare_parameter<double>("depth_to_color_tz"));
    detector_->set_depth_color_extrinsic(depth_color_extrinsic_);

    color_props_.intr.fx = declare_parameter<double>("color_fx");
    color_props_.intr.fy = declare_parameter<double>("color_fy");
    color_props_.intr.cx = declare_parameter<double>("color_cx");
    color_props_.intr.cy = declare_parameter<double>("color_cy");
    color_props_.dim.width = declare_parameter<int>("color_image_width");
    color_props_.dim.height = declare_parameter<int>("color_image_height");
    color_props_.intr.dist[0] = declare_parameter<double>("color_d1");
    color_props_.intr.dist[1] = declare_parameter<double>("color_d2");
    color_props_.intr.dist[2] = declare_parameter<double>("color_d3");
    color_props_.intr.dist[3] = declare_parameter<double>("color_d4");
    color_props_.intr.dist[4] = declare_parameter<double>("color_d5");
    detector_->set_color_image_properties(color_props_);
    detector_->calculate_letterbox_padding();
    RCLCPP_INFO(get_logger(),
                "Color intrinsics loaded from config as fallback "
                "(fx=%.2f fy=%.2f cx=%.2f cy=%.2f)",
                color_props_.intr.fx, color_props_.intr.fy,
                color_props_.intr.cx, color_props_.intr.cy);

    depth_props_.intr.fx = declare_parameter<double>("depth_fx");
    depth_props_.intr.fy = declare_parameter<double>("depth_fy");
    depth_props_.intr.cx = declare_parameter<double>("depth_cx");
    depth_props_.intr.cy = declare_parameter<double>("depth_cy");
    depth_props_.dim.width = declare_parameter<int>("depth_image_width");
    depth_props_.dim.height = declare_parameter<int>("depth_image_height");
    detector_->set_depth_image_properties(depth_props_);
    RCLCPP_INFO(get_logger(),
                "Depth intrinsics loaded from config as fallback "
                "(fx=%.2f fy=%.2f cx=%.2f cy=%.2f)",
                depth_props_.intr.fx, depth_props_.intr.fy,
                depth_props_.intr.cx, depth_props_.intr.cy);
}

void ValvePoseNode::setup_publishers(const rclcpp::QoS& qos) {
    const auto lm_topic = declare_parameter<std::string>("landmarks_pub_topic");
    landmark_pub_ =
        create_publisher<vortex_msgs::msg::LandmarkArray>(lm_topic, qos);

    if (!debug_visualize_)
        return;

    const auto pose_topic =
        declare_parameter<std::string>("debug.valve_poses_pub_topic");
    const auto depth_cloud_topic =
        declare_parameter<std::string>("debug.depth_cloud_pub_topic");
    const auto depth_color_topic =
        declare_parameter<std::string>("debug.depth_colormap_pub_topic");
    const auto ann_topic =
        declare_parameter<std::string>("debug.annulus_pub_topic");
    const auto pln_topic =
        declare_parameter<std::string>("debug.plane_pub_topic");
    depth_colormap_vmin_ = static_cast<float>(
        declare_parameter<double>("debug.depth_colormap_value_min"));
    depth_colormap_vmax_ = static_cast<float>(
        declare_parameter<double>("debug.depth_colormap_value_max"));

    pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseArray>(pose_topic, qos);
    depth_cloud_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(depth_cloud_topic, qos);
    depth_colormap_pub_ =
        create_publisher<sensor_msgs::msg::Image>(depth_color_topic, qos);
    annulus_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(ann_topic, qos);
    plane_pub_ =
        create_publisher<sensor_msgs::msg::PointCloud2>(pln_topic, qos);
}

void ValvePoseNode::setup_subscribers(const rclcpp::QoS& qos) {
    const auto depth_topic =
        declare_parameter<std::string>("depth_image_sub_topic");
    const auto det_topic =
        declare_parameter<std::string>("detections_sub_topic");
    const auto depth_info_topic =
        declare_parameter<std::string>("depth_image_info_topic");
    const auto color_info_topic =
        declare_parameter<std::string>("color_image_info_topic");

    const auto info_qos =
        rclcpp::QoS(rclcpp::KeepLast(1))
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
            .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    color_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        color_info_topic, info_qos,
        std::bind(&ValvePoseNode::color_camera_info_cb, this, _1));
    depth_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, info_qos,
        std::bind(&ValvePoseNode::depth_camera_info_cb, this, _1));

    depth_sub_.subscribe(this, depth_topic, qos.get_rmw_qos_profile());
    det_sub_.subscribe(this, det_topic, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), depth_sub_, det_sub_);
    sync_->registerCallback(std::bind(&ValvePoseNode::sync_cb, this, _1, _2));
}

// One-shot callback that overrides color intrinsics and distortion from the
// camera_info topic.
void ValvePoseNode::color_camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    color_props_.intr.fx = msg->k[0];
    color_props_.intr.fy = msg->k[4];
    color_props_.intr.cx = msg->k[2];
    color_props_.intr.cy = msg->k[5];
    color_props_.dim.width = static_cast<int>(msg->width);
    color_props_.dim.height = static_cast<int>(msg->height);

    if (msg->d.size() >= 5) {
        for (size_t i = 0; i < 5; ++i)
            color_props_.intr.dist[i] = msg->d[i];
    }

    detector_->set_color_image_properties(color_props_);
    detector_->calculate_letterbox_padding();

    color_cam_info_sub_.reset();  // one-shot
    RCLCPP_INFO(get_logger(),
                "Color camera_info received, overriding config fallback "
                "(fx=%.2f fy=%.2f cx=%.2f cy=%.2f)",
                color_props_.intr.fx, color_props_.intr.fy,
                color_props_.intr.cx, color_props_.intr.cy);
}

// One-shot callback that overrides depth intrinsics from the camera_info topic.
void ValvePoseNode::depth_camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    depth_props_.intr.fx = msg->k[0];
    depth_props_.intr.fy = msg->k[4];
    depth_props_.intr.cx = msg->k[2];
    depth_props_.intr.cy = msg->k[5];
    depth_props_.dim.width = static_cast<int>(msg->width);
    depth_props_.dim.height = static_cast<int>(msg->height);

    detector_->set_depth_image_properties(depth_props_);

    depth_cam_info_sub_.reset();  // one-shot
    RCLCPP_INFO(get_logger(), "Depth camera_info received (fx=%.1f fy=%.1f)",
                depth_props_.intr.fx, depth_props_.intr.fy);
}

// Main synchronized callback: runs NMS, estimates poses, and publishes all
// outputs.
void ValvePoseNode::sync_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det) {
    if (!depth || !det)
        return;

    cv::Mat depth_color;
    const bool publish_colormap =
        debug_visualize_ && depth_colormap_pub_ &&
        depth_colormap_pub_->get_subscription_count() > 0;
    if (publish_colormap) {
        cv_bridge::CvImageConstPtr cv_raw =
            cv_bridge::toCvShare(depth, "16UC1");
        cv::Mat depth_f;
        cv_raw->image.convertTo(depth_f, CV_32FC1);
        const float scale =
            255.0f / (depth_colormap_vmax_ - depth_colormap_vmin_);
        cv::Mat depth_8u;
        depth_f.convertTo(depth_8u, CV_8UC1, scale,
                          -depth_colormap_vmin_ * scale);
        cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_TURBO);
    }

    if (det->detections.empty()) {
        // Publish empty arrays to clear stale data from previous detections.
        if (debug_visualize_ && pose_pub_)
            pose_pub_->publish(make_pose_array({}, depth->header));
        landmark_pub_->publish(
            make_landmark_array({}, depth->header, landmark_type_));
        if (publish_colormap) {
            depth_colormap_pub_->publish(
                *cv_bridge::CvImage(depth->header, "bgr8", depth_color)
                     .toImageMsg());
        }
        return;
    }

    std::vector<std::pair<float, BoundingBox>> scored_boxes;
    scored_boxes.reserve(det->detections.size());
    for (const auto& d : det->detections) {
        float score = 0.0f;
        if (!d.results.empty()) {
            score = static_cast<float>(d.results[0].hypothesis.score);
        } else {
            score = static_cast<float>(d.bbox.size_x * d.bbox.size_y);
        }
        scored_boxes.emplace_back(score, to_bbox(d.bbox));
    }
    const std::vector<size_t> kept =
        filter_duplicate_detections(scored_boxes, iou_duplicate_threshold_);

    const cv::Mat depth_img = decode_depth_to_float(depth);

    pcl::PointCloud<pcl::PointXYZ>::Ptr ann_dbg(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pln_dbg(
        new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<BoundingBox> raw_boxes;
    std::vector<Pose> poses;

    for (size_t idx : kept) {
        const BoundingBox& yolo_box = scored_boxes[idx].second;
        BoundingBox org_box = undistort_bbox(yolo_box, color_props_.intr);
        raw_boxes.push_back(yolo_box);

        const auto pose_result = detector_->compute_pose_from_depth(
            depth_img, org_box, ann_dbg, pln_dbg, true);
        if (pose_result.result_valid)
            poses.push_back(pose_result.result);
    }

    if (debug_visualize_ && depth_cloud_pub_ &&
        depth_cloud_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*ann_dbg, cloud_msg);
        cloud_msg.header = depth->header;
        depth_cloud_pub_->publish(cloud_msg);
    }

    if (publish_colormap) {
        for (size_t i = 0; i < raw_boxes.size(); ++i) {
            const auto& box = raw_boxes[i];
            const float Z = (i < poses.size() && poses[i].z > 0.0)
                                ? static_cast<float>(poses[i].z)
                                : 0.0f;

            const float angle_deg =
                box.theta * 180.0f / static_cast<float>(M_PI);
            cv::RotatedRect rrect(cv::Point2f(box.center_x, box.center_y),
                                  cv::Size2f(box.size_x, box.size_y),
                                  angle_deg);
            cv::Point2f corners[4];
            rrect.points(corners);

            if (Z > 0.0f) {
                for (auto& c : corners) {
                    c = project_color_pixel_to_depth(c.x, c.y, Z, color_props_,
                                                     depth_props_,
                                                     depth_color_extrinsic_);
                }
            }
            for (int j = 0; j < 4; ++j) {
                cv::line(depth_color, corners[j], corners[(j + 1) % 4],
                         cv::Scalar(0, 255, 0), 2);
            }
        }
        depth_colormap_pub_->publish(
            *cv_bridge::CvImage(depth->header, "bgr8", depth_color)
                 .toImageMsg());
    }

    if (debug_visualize_ && annulus_pub_ && plane_pub_) {
        sensor_msgs::msg::PointCloud2 ann_msg, pln_msg;
        pcl::toROSMsg(*ann_dbg, ann_msg);
        pcl::toROSMsg(*pln_dbg, pln_msg);
        ann_msg.header = depth->header;
        pln_msg.header = depth->header;
        annulus_pub_->publish(ann_msg);
        plane_pub_->publish(pln_msg);
    }

    std_msgs::msg::Header pose_header = depth->header;
    if (!output_frame_id_.empty())
        pose_header.frame_id = output_frame_id_;

    if (debug_visualize_ && pose_pub_)
        pose_pub_->publish(make_pose_array(poses, pose_header));
    landmark_pub_->publish(
        make_landmark_array(poses, pose_header, landmark_type_));
}

}  // namespace valve_detection

RCLCPP_COMPONENTS_REGISTER_NODE(valve_detection::ValvePoseNode)
