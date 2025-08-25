#include "valve_detection_ros/valve_pose_ros.hpp"

namespace valve_detection {

using namespace std::placeholders;

ValvePoseNode::ValvePoseNode(const rclcpp::NodeOptions& options)
    : Node("valve_detection_node", options) {
    init_valve_detector();

    setup_sync();

    if (calculate_angle_) {
        init_angle_detector();
    }

    std::string color_image_info_topic =
        this->declare_parameter<std::string>("color_image_info_topic");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    color_image_info_sub_ =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            color_image_info_topic, qos,
            std::bind(&ValvePoseNode::color_image_info_callback, this,
                      std::placeholders::_1));

    std::string valve_poses_pub_topic =
        this->declare_parameter<std::string>("valve_poses_pub_topic");

    valve_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        valve_poses_pub_topic, qos);

    pcl_visualize_ = this->declare_parameter<bool>("pcl_visualize");

    if (pcl_visualize_) {
        std::string annulus_pub_topic =
            this->declare_parameter<std::string>("annulus_pub_topic");
        annulus_pcl_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                annulus_pub_topic, qos);
        std::string plane_pub_topic =
            this->declare_parameter<std::string>("plane_pub_topic");
        annulus_plane_pub_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                plane_pub_topic, qos);
    }
    if (visualize_detections_) {
        std::string processed_image_pub_topic =
            this->declare_parameter<std::string>("processed_image_pub_topic");
        processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            processed_image_pub_topic, qos);
    }
    if (!use_depth_image_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
}

void ValvePoseNode::init_valve_detector() {
    int yolo_img_width = this->declare_parameter<int>("yolo_img_width");
    int yolo_img_height = this->declare_parameter<int>("yolo_img_height");

    float annulus_radius_ratio =
        this->declare_parameter<float>("annulus_radius_ratio");

    float plane_ransac_threshold =
        this->declare_parameter<float>("plane_ransac_threshold");
    int plane_ransac_max_iterations =
        this->declare_parameter<int>("plane_ransac_max_iterations");

    float valve_handle_offset =
        this->declare_parameter<float>("valve_handle_offset");

    valve_detector_ = std::make_unique<ValveDetector>(
        yolo_img_width, yolo_img_height, annulus_radius_ratio,
        plane_ransac_threshold, plane_ransac_max_iterations,
        valve_handle_offset);
}

void ValvePoseNode::setup_sync() {
    visualize_detections_ =
        this->declare_parameter<bool>("visualize_detections");
    calculate_angle_ = this->declare_parameter<bool>("calculate_angle");
    use_depth_image_ = this->declare_parameter<bool>("use_depth_image");

    use_color_image_ = (calculate_angle_ || visualize_detections_);

    std::string color_image_sub_topic =
        this->declare_parameter<std::string>("color_image_sub_topic");
    std::string depth_image_sub_topic =
        this->declare_parameter<std::string>("depth_image_sub_topic");
    std::string pcl_sub_topic =
        this->declare_parameter<std::string>("pcl_sub_topic");
    std::string detections_sub_topic =
        this->declare_parameter<std::string>("detections_sub_topic");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    if (use_color_image_ && use_depth_image_) {
        color_image_sub_.subscribe(this, color_image_sub_topic,
                                   qos.get_rmw_qos_profile());
        depth_image_sub_.subscribe(this, depth_image_sub_topic,
                                   qos.get_rmw_qos_profile());
        detections_sub_.subscribe(this, detections_sub_topic,
                                  qos.get_rmw_qos_profile());
        sync_di_ci_d_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy_DI_CI_D>>(
                SyncPolicy_DI_CI_D(10), depth_image_sub_, color_image_sub_,
                detections_sub_);
        sync_di_ci_d_->registerCallback(
            std::bind(&ValvePoseNode::di_ci_d_callback, this, _1, _2, _3));

    } else if (use_depth_image_) {
        depth_image_sub_.subscribe(this, depth_image_sub_topic,
                                   qos.get_rmw_qos_profile());
        detections_sub_.subscribe(this, detections_sub_topic,
                                  qos.get_rmw_qos_profile());
        sync_di_d_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy_DI_D>>(
                SyncPolicy_DI_D(10), depth_image_sub_, detections_sub_);
        sync_di_d_->registerCallback(
            std::bind(&ValvePoseNode::di_d_callback, this, _1, _2));

    } else if (use_color_image_ && !use_depth_image_) {
        pcl_sub_.subscribe(this, pcl_sub_topic, qos.get_rmw_qos_profile());
        color_image_sub_.subscribe(this, color_image_sub_topic,
                                   qos.get_rmw_qos_profile());
        detections_sub_.subscribe(this, detections_sub_topic,
                                  qos.get_rmw_qos_profile());
        sync_pc_ci_d_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy_PC_CI_D>>(
                SyncPolicy_PC_CI_D(10), pcl_sub_, color_image_sub_,
                detections_sub_);
        sync_pc_ci_d_->registerCallback(
            std::bind(&ValvePoseNode::pc_ci_d_callback, this, _1, _2, _3));

    } else {
        pcl_sub_.subscribe(this, pcl_sub_topic, qos.get_rmw_qos_profile());
        detections_sub_.subscribe(this, detections_sub_topic,
                                  qos.get_rmw_qos_profile());
        sync_pc_d_ =
            std::make_shared<message_filters::Synchronizer<SyncPolicy_PC_D>>(
                SyncPolicy_PC_D(10), pcl_sub_, detections_sub_);
        sync_pc_d_->registerCallback(
            std::bind(&ValvePoseNode::pc_d_callback, this, _1, _2));
    }
}

void ValvePoseNode::di_ci_d_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    synchronized_callback<sensor_msgs::msg::Image>(depth_image, color_image,
                                                   detections);
}

void ValvePoseNode::di_d_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    synchronized_callback<sensor_msgs::msg::Image>(depth_image, nullptr,
                                                   detections);
}

void ValvePoseNode::pc_ci_d_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl,
    const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    synchronized_callback<sensor_msgs::msg::PointCloud2>(pcl, color_image,
                                                         detections);
}

void ValvePoseNode::pc_d_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    synchronized_callback<sensor_msgs::msg::PointCloud2>(pcl, nullptr,
                                                         detections);
}

void ValvePoseNode::init_angle_detector() {
    if (calculate_angle_) {
        AngleDetectorParams angle_params;
        angle_params.line_detection_area =
            this->declare_parameter<float>("angle.detection_area_ratio");
        angle_params.canny_low_threshold =
            this->declare_parameter<int>("angle.canny_low_threshold");
        angle_params.canny_high_threshold =
            this->declare_parameter<int>("angle.canny_high_threshold");
        angle_params.canny_aperture_size =
            this->declare_parameter<int>("angle.canny_aperture_size");
        angle_params.hough_rho_res =
            this->declare_parameter<double>("angle.hough_rho_res");
        angle_params.hough_theta_res =
            this->declare_parameter<double>("angle.hough_theta_res");
        angle_params.hough_threshold =
            this->declare_parameter<int>("angle.hough_threshold");
        angle_params.hough_min_line_length =
            this->declare_parameter<double>("angle.hough_min_line_length");
        angle_params.hough_max_line_gap =
            this->declare_parameter<double>("angle.hough_max_line_gap");

        valve_detector_->init_angle_detector(angle_params);
    }
}

void ValvePoseNode::color_image_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg) {
    if (!color_image_info_received_) {
        ImageProperties img_props;
        img_props.intr.fx = camera_info_msg->k[0];
        img_props.intr.fy = camera_info_msg->k[4];
        img_props.intr.cx = camera_info_msg->k[2];
        img_props.intr.cy = camera_info_msg->k[5];
        img_props.dim.width = camera_info_msg->width;
        img_props.dim.height = camera_info_msg->height;

        valve_detector_->set_color_image_properties(img_props);
        valve_detector_->calculate_letterbox_padding();

        color_image_frame_id_ = camera_info_msg->header.frame_id;
        color_image_info_received_ = true;
        color_image_info_sub_.reset();
    }
}

BoundingBox ValvePoseNode::to_bounding_box(
    const vision_msgs::msg::BoundingBox2D& bbox) const {
    BoundingBox box;
    box.center_x = bbox.center.position.x;
    box.center_y = bbox.center.position.y;
    box.size_x = bbox.size_x;
    box.size_y = bbox.size_y;
    box.theta = bbox.center.theta;
    return box;
}

void ValvePoseNode::publish_annulus_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std_msgs::msg::Header& header) const {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.is_dense = false;
    cloud_msg.width = static_cast<uint32_t>(cloud->points.size());
    cloud_msg.height = 1;

    annulus_pcl_pub_->publish(cloud_msg);
}

void ValvePoseNode::publish_annulus_plane(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std_msgs::msg::Header& header) const {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    cloud_msg.is_dense = false;
    cloud_msg.width = static_cast<uint32_t>(cloud->points.size());
    cloud_msg.height = 1;

    annulus_plane_pub_->publish(cloud_msg);
}

void ValvePoseNode::publish_valve_poses(
    const std::vector<Pose>& poses,
    const std_msgs::msg::Header& header) const {
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header = header;

    for (const auto& pose : poses) {
        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = pose.position.x();
        pose_msg.position.y = pose.position.y();
        pose_msg.position.z = pose.position.z();
        pose_msg.orientation.x = pose.orientation.x();
        pose_msg.orientation.y = pose.orientation.y();
        pose_msg.orientation.z = pose.orientation.z();
        pose_msg.orientation.w = pose.orientation.w();

        pose_array_msg.poses.push_back(pose_msg);
    }
    valve_poses_pub_->publish(pose_array_msg);
}

}  // namespace valve_detection

RCLCPP_COMPONENTS_REGISTER_NODE(valve_detection::ValvePoseNode)
