#include "valve_detection/valve_pose_depth_ros.hpp"

ValvePoseDepthNode::ValvePoseDepthNode(const rclcpp::NodeOptions& options)
: Node("valve_detection_node", options) {
    std::string color_image_sub_topic =
        this->declare_parameter<std::string>("color_image_sub_topic");
    std::string depth_image_sub_topic =
        this->declare_parameter<std::string>("depth_image_sub_topic");
    std::string detections_sub_topic =
        this->declare_parameter<std::string>("detections_sub_topic");
    visualize_detections_ =
        this->declare_parameter<bool>("visualize_detections");
    debug_visualize_ =
        this->declare_parameter<bool>("debug_visualize");
    calculate_angle_ =
        this->declare_parameter<bool>("calculate_angle");

    int yolo_img_width =
        this->declare_parameter<int>("yolo_img_width");
    int yolo_img_height =
        this->declare_parameter<int>("yolo_img_height");

    float annulus_radius_ratio =
        this->declare_parameter<float>("annulus_radius_ratio");
    
    float plane_ransac_threshold =
        this->declare_parameter<float>("plane_ransac_threshold");
    int plane_ransac_max_iterations =
        this->declare_parameter<int>("plane_ransac_max_iterations");

    float valve_handle_offset =
        this->declare_parameter<float>("valve_handle_offset");

    valve_detector_ = std::make_shared<ValvePoseDepth>(yolo_img_width, yolo_img_height, annulus_radius_ratio,
        plane_ransac_threshold, plane_ransac_max_iterations, valve_handle_offset);


    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    depth_image_sub_.subscribe(this, depth_image_sub_topic,
                               qos.get_rmw_qos_profile());
    color_image_sub_.subscribe(this, color_image_sub_topic,
                               qos.get_rmw_qos_profile());
    detections_sub_.subscribe(this, detections_sub_topic,
                              qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
        MySyncPolicy(10), depth_image_sub_, color_image_sub_, detections_sub_);
    sync_->registerCallback(std::bind(
        &ValvePoseDepthNode::synchronized_callback, this, _1, _2, _3));

    std::string color_image_info_topic =
        this->declare_parameter<std::string>("color_image_info_topic");

    qos.history = rclcpp::KeepLast(1);
        color_image_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            color_image_info_topic, 10,
            std::bind(&ValvePoseDepthNode::color_image_info_callback,
                      this, std::placeholders::_1));

    std::string valve_pose_pub_topic =
        this->declare_parameter<std::string>("valve_pose_pub_topic");

    valve_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        valve_pose_pub_topic, qos);

    if (debug_visualize_) {
        std::string annulus_pub_topic =
            this->declare_parameter<std::string>("annulus_pub_topic");
        annulus_pcl_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            annulus_pub_topic, qos);
    }
}

void ValvePoseDepthNode::color_image_info_callback(
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg) {
    if (!color_image_info_received_) {
        ImageProperties img_props;
        img_props.intr.fx = camera_info_msg->k[0];
        img_props.intr.fy = camera_info_msg->k[4];
        img_props.intr.cx = camera_info_msg->k[2];
        img_props.intr.cy = camera_info_msg->k[5];
        img_props.dim.x = camera_info_msg->width;
        img_props.dim.y = camera_info_msg->height;

        valve_detector_->set_color_image_properties(img_props);
        valve_detector_->calculate_letterbox_padding();

        color_image_frame_id_ = camera_info_msg->header.frame_id;
        color_image_info_received_ = true;
        color_image_info_sub_.reset();
    }
}

BoundingBox ValvePoseDepthNode::to_bounding_box(
    const vision_msgs::msg::BoundingBox2D& bbox) const {
    BoundingBox box;
    box.center_x = bbox.center.x;
    box.center_y = bbox.center.y;
    box.size_x = bbox.size_x;
    box.size_y = bbox.size_y;
    box.theta = bbox.center.theta;
    return box;
}

void ValvePoseDepthNode::publish_annulus_pcl(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) const {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = color_image_frame_id_;
    cloud_msg.header.stamp = this->now();
    cloud_msg.is_dense = false;
    cloud_msg.width = static_cast<uint32_t>(cloud->points.size());
    cloud_msg.height = 1;

    annulus_pcl_pub_->publish(cloud_msg);
}

void ValvePoseDepthNode::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr& color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detections) {
    if (!color_image_info_received_) {
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptr =
        cv_bridge::toCvShare(color_image, "bgr8");
    
    cv:brudge::CvImageConstPtr cv_depth_ptr =
        cv_bridge::toCvShare(depth_image, "32FC1");

    cv::Mat cv_color_image = cv_ptr->image;

    if (visualize_detections_) {
        cv_color_image = cv_color_image.clone();
    }
    std::vector<BoundingBox> boxes = std::vector<BoundingBox>().reserve(detections->detections.size());
    for (const auto& detection : detections->detections) {
        boxes.push_back(to_bounding_box(detection.bbox));
    }

    for (const auto& box : boxes) {
        BoundingBox org_image_box =
            valve_detector_->transform_bounding_box(box);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);

        valve_detector_->extract_annulus_pcl(
            cv_depth_ptr->image,
            org_image_box,
            cloud);

        if (cloud->empty()) {
            continue;
        }
        if (debug_visualize_) {
            publish_annulus_pcl(cloud);
        }

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        if (!valve_detector_->segment_plane(cloud, coefficients, inliers)) {
            continue;
        }

        Eigen::Vector3f bb_centre_ray =
            valve_detector_->get_ray_direction(org_image_box);

        Eigen::Vector3f ray_plane_intersection =
            valve_detector_->find_ray_plane_intersection(coefficients, bb_centre_ray);
        
        if (ray_plane_intersection.isZero()) {
            continue;
        }

        Eigen::Vector3f plane_normal =
            valve_detector_->compute_plane_normal(
                coefficients, ray_direction);

        if (plane_normal.isZero()) {
            continue;
        }

        Eigen::Vector3f shifted_position =
            valve_detector_->shift_point_along_normal(
                ray_plane_intersection, plane_normal);

        float angle = org_image_box.theta;
        if (calculate_angle_) {
            // angle = calculate_angle() 
        }
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = color_image->header.stamp;
        pose_msg.header.frame_id = color_image_frame_id_;

        pose_msg.pose.position.x = shifted_position[0];
        pose_msg.pose.position.y = shifted_position[1];
        pose_msg.pose.position.z = shifted_position[2];

    }





}

RCLCPP_COMPONENTS_REGISTER_NODE(ValvePoseDepthNode)