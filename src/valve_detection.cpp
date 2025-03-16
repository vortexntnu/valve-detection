#include <valve_detection/valve_detection.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ValveDetectionNode::ValveDetectionNode(const rclcpp::NodeOptions & options)
    : Node("valve_detection_node", options)
{
    auto depth_image_sub_topic = this->declare_parameter<std::string>("depth_image_sub_topic");
    auto color_image_sub_topic = this->declare_parameter<std::string>("color_image_sub_topic");
    auto detections_sub_topic = this->declare_parameter<std::string>("detections_sub_topic");

    line_detection_area_ = this->declare_parameter<int>("line_detection_area", 5);

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    depth_image_sub_.subscribe(this, depth_image_sub_topic, qos.get_rmw_qos_profile());
    color_image_sub_.subscribe(this, color_image_sub_topic, qos.get_rmw_qos_profile());
    detections_sub_.subscribe(this, detections_sub_topic, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), depth_image_sub_, color_image_sub_, detections_sub_);
    sync_->registerCallback(std::bind(&ValveDetectionNode::synchronized_callback, this, _1, _2, _3));

    // Create subscriptions with synchronized callbacks
    camera_info_subscription_yolo_color_  = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/yolov8_encoder/resize/camera_info", 10,
        std::bind(&ValveDetectionNode::camera_info_callback_yolo_colored, this, std::placeholders::_1));

    // Create subscriptions with synchronized callbacks
    camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/zed_node/depth/camera_info", 10,
        std::bind(&ValveDetectionNode::camera_info_callback, this, std::placeholders::_1));

    processed_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("yolov8_valve_detection_image", 10);
    plane_normal_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("plane_normal", 10);
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    line_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("valve_pose", 10);
    line_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_points", 10);
    near_plane_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("near_plane_cloud", 10);  
}

Eigen::Vector3f ValveDetectionNode::filter_direction_ = Eigen::Vector3f(1, 0, 0);

void ValveDetectionNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
    if (!camera_info_received_)
    {
        camera_info_ = camera_info_msg;
        camera_info_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera resize info received and stored.");
        
        compute_height_width_scalars();
        // unsubscribe to avoid further callback executions
        camera_info_subscription_.reset();
    }
}

void ValveDetectionNode::camera_info_callback_yolo_colored(const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
    if (!camera_info_received_yolo_color_)
    {
        camera_info_yolo_color_ = camera_info_msg;
        camera_info_received_yolo_color_ = true;
        RCLCPP_INFO(this->get_logger(), "Camera yolo color info received and stored.");
        
        compute_height_width_scalars();
        // unsubscribe to avoid further callback executions
        camera_info_subscription_yolo_color_.reset();
    }
}

void ValveDetectionNode::compute_height_width_scalars()
{
    if (camera_info_received_ && camera_info_received_yolo_color_)
    {
        height_scalar_ = static_cast<double>(camera_info_yolo_color_->height -280) / camera_info_->height;
        width_scalar_ = static_cast<double>(camera_info_yolo_color_->width) / camera_info_->width;


        RCLCPP_INFO(this->get_logger(), "Height scalar: %.3f", height_scalar_);
        RCLCPP_INFO(this->get_logger(), "Width scalar: %.3f", width_scalar_);
    }
}

void ValveDetectionNode::project_pixel_to_3d(int u, int v, float depth, pcl::PointXYZ &point)
{
    if (!camera_info_ || depth <= 0)
    {
        return;  // Skip invalid depth or missing camera info
    }

    // Extract intrinsic parameters
    double fx = camera_info_->k[0];  // k[0] = fx
    double fy = camera_info_->k[4];  // k[4] = fy
    double cx = camera_info_->k[2];  // k[2] = cx
    double cy = camera_info_->k[5];  // k[5] = cy

    // Calculate 3D point coordinates
    // transform from optical to camera frame
    point.x = depth;
    point.y = -(u - cx) * depth / fx;
    point.z = -(v - cy) * depth / fy;
}

void ValveDetectionNode::project_3d_to_pixel(float x, float y, float z, int &u, int &v)
{
    if (!camera_info_ || z <= 0) {
        return; // Ensure camera info is available and z is positive
    }

    // Extract intrinsic parameters
    double fx = camera_info_->k[0];  // Focal length in x direction
    double fy = camera_info_->k[4];  // Focal length in y direction
    double cx = camera_info_->k[2];  // Principal point in x direction
    double cy = camera_info_->k[5];  // Principal point in y direction

    // Project the 3D point to 2D pixel coordinates
    // transform from camera to optical frame
    u = static_cast<int>((-y * fx) / x + cx);
    v = static_cast<int>((-z * fy) / x + cy);
}

void ValveDetectionNode::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections)
{

    RCLCPP_INFO(this->get_logger(), "Synchronized image and detection messages received.");

    process_and_publish_image(depth_image, color_image, detections);
    
}

void ValveDetectionNode::process_and_publish_image(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_image,
    const sensor_msgs::msg::Image::ConstSharedPtr & color_image,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detections)
{
    if (!camera_info_received_ || !camera_info_received_yolo_color_)
    {
        return;
    }

    try
    {
        // Convert depth and color images
        cv::Mat cv_depth_image = cv_bridge::toCvCopy(depth_image, "32FC1")->image;
        cv::Mat cv_color_image = cv_bridge::toCvCopy(color_image, "bgr8")->image;

        if (!detections->detections.empty())
        {
            const auto *first_detection = &detections->detections[0];

            // The most centred of the two most likely variables are chosen as a boundingbox.
            // Calculate distance to center (cx, cy)
            double cx = camera_info_->k[2];  
            double cy = camera_info_->k[5];  
            if (detections->detections.size() > 1)
            {
                float dist_first = std::sqrt(std::pow(detections->detections[0].bbox.center.position.x - cx, 2) + 
                                            std::pow(detections->detections[0].bbox.center.position.y - cy, 2));

                float dist_second = std::sqrt(std::pow(detections->detections[1].bbox.center.position.x - cx, 2) + 
                                            std::pow(detections->detections[1].bbox.center.position.y - cy, 2));

                // Select the detection that is closer to the center.
                if (dist_first > dist_second) {
                    first_detection = &detections->detections[1];
                }
            }

            int center_x = static_cast<int>(first_detection->bbox.center.position.x / this->width_scalar_);
            int center_y = static_cast<int>((first_detection->bbox.center.position.y - 140) / this->height_scalar_);
            int width = static_cast<int>(first_detection->bbox.size_x / this->width_scalar_);
            int height = static_cast<int>(first_detection->bbox.size_y / this->height_scalar_);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            bool points_added = false;

            for (int y = center_y - (height / 2); y <= center_y + (height / 2); ++y)
            {
                for (int x = center_x - (width / 2); x <= center_x + (width / 2); ++x)
                {
                    int dx = x - center_x;
                    int dy = y - center_y;
                    if (dx * dx / (float)((width / 2) * (width / 2)) + dy * dy / (float)((height / 2) * (height / 2)) <= 1.0)
                    {
                        if (y >= 0 && y < cv_depth_image.rows && x >= 0 && x < cv_depth_image.cols)
                        {
                            float depth_value = cv_depth_image.at<float>(y, x);
                            pcl::PointXYZ point;
                            project_pixel_to_3d(x, y, depth_value, point);

                            if (depth_value > 0) // Check for valid depth
                            {
                                cloud->points.push_back(point);
                                points_added = true;
                            }
                        }
                    }
                }
            }

            if (points_added && cloud->points.size() > 0)
            {




                sensor_msgs::msg::PointCloud2 cloud_msg;
                pcl::toROSMsg(*cloud, cloud_msg);
                cloud_msg.header.stamp = depth_image->header.stamp;
                cloud_msg.header.frame_id = "zed_left_camera_frame";

                pointcloud_pub_->publish(cloud_msg);

                pcl::SACSegmentation<pcl::PointXYZ> seg;
                seg.setOptimizeCoefficients(true);
                seg.setModelType(pcl::SACMODEL_PLANE);
                seg.setMethodType(pcl::SAC_RANSAC);
                seg.setDistanceThreshold(0.01);

                pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
                seg.setInputCloud(cloud);
                seg.segment(*inliers, *coefficients);

                if (inliers->indices.size() > 0)
                {
                    geometry_msgs::msg::Vector3 normal_msg;
                    normal_msg.x = coefficients->values[0];
                    normal_msg.y = coefficients->values[1];
                    normal_msg.z = coefficients->values[2];
                    plane_normal_pub_->publish(normal_msg);

                    RCLCPP_INFO(this->get_logger(), "RANSAC successful: Plane normal (%.3f, %.3f, %.3f)",
                                 coefficients->values[0], coefficients->values[1], coefficients->values[2]);

                    pcl::PointCloud<pcl::PointXYZ>::Ptr near_plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                    // Select points within a distance threshold of the plane model
                    pcl::ExtractIndices<pcl::PointXYZ> extract;
                    extract.setInputCloud(cloud);
                    extract.setIndices(inliers);
                    extract.setNegative(false);  // Select the inliers
                    extract.filter(*near_plane_cloud);


                    // Now we can use `selectWithinDistance` to select inliers that are within the distance threshold
                    pcl::PointIndices::Ptr selected_inliers(new pcl::PointIndices());
                    float threshold_distance = 0.02;  // Example threshold (3 cm)

                    // Logic for selecting points close to the threshold_distance of the plane.
                    for (size_t i = 0; i < cloud->points.size(); ++i) {
                        const auto& point = cloud->points[i];
                        float distance = std::abs(coefficients->values[0] * point.x +
                                                coefficients->values[1] * point.y +
                                                coefficients->values[2] * point.z +
                                                coefficients->values[3]) /
                                        std::sqrt(coefficients->values[0] * coefficients->values[0] +
                                                coefficients->values[1] * coefficients->values[1] +
                                                coefficients->values[2] * coefficients->values[2]);
                        if (distance < threshold_distance) {
                            selected_inliers->indices.push_back(i);
                        }
                    }
                    RCLCPP_INFO(this->get_logger(), "Selected inlier size: %zu", selected_inliers->indices.size());

                    // Extract the selected inliers
                    pcl::PointCloud<pcl::PointXYZ>::Ptr final_inliers_cloud(new pcl::PointCloud<pcl::PointXYZ>);

                    for (const auto& idx : selected_inliers->indices)
                    {
                            // Convert 3D point (x, y) to image space (u, v)
                            int u, v;
                            const auto& point = cloud->points[idx];
                            project_3d_to_pixel(point.x, point.y, point.z, u, v);


                            // Check if the point (u, v) is inside the smaller ellipse
                            int dx = u - center_x;
                            int dy = v - center_y;

                            // Check if point is inside the smaller ellipse in image space
                            float a = static_cast<float>(width) / 6.0f;  // Semi-major axis
                            float b = static_cast<float>(height) / 6.0f; // Semi-minor axis

                            if ((dx * dx) / (a * a) + (dy * dy) / (b * b) <= 1.0f) {
                                // Point is inside the ellipse, add it to the near_plane_cloud
                                final_inliers_cloud->points.push_back(cloud->points[idx]);
                            }

                        
                    }

                    near_plane_cloud = final_inliers_cloud;

                // Calculate centroid position from near_plane_cloud
                Eigen::Vector3f centroid(0, 0, 0);
                if (!near_plane_cloud->empty()) {
                    for (const auto& point : near_plane_cloud->points) {
                        centroid += Eigen::Vector3f(point.x, point.y, point.z);
                    }
                    centroid /= near_plane_cloud->points.size();
                } else {
                    RCLCPP_WARN(this->get_logger(), "No valid near plane points found.");
                    return;
                }

                int x1 = std::max(center_x - width * line_detection_area_/10 , 0);
                int y1 = std::max(center_y - height * line_detection_area_/10 , 0);
                int x2 = std::min(center_x + width * line_detection_area_/10 , cv_color_image.cols - 1);
                int y2 = std::min(center_y + height * line_detection_area_/10 , cv_color_image.rows - 1);
                // Draw roi for debugging
                cv::rectangle(cv_color_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

                // Extract ROI (region of interest) from the color image
                cv::Mat roi = cv_color_image(cv::Rect(x1, y1, x2 - x1, y2 - y1));

                // Convert to grayscale for edge detection
                cv::Mat gray;
                cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

                // Apply Canny edge detection
                cv::Mat edges;
                cv::Canny(gray, edges, 50, 150, 3);

                // Detect lines using Hough Transform
                std::vector<cv::Vec4i> lines;
                cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 20, 40, 10);

                if (!lines.empty())
                {
                    cv::Vec4i longest_line = lines[0];
                    double max_length = 0;

                    for (const auto &line : lines)
                    {
                        double length = std::hypot(line[2] - line[0], line[3] - line[1]);
                        if (length > max_length)
                        {
                            max_length = length;
                            longest_line = line;
                        }
                    }

                    double angle = std::atan2(longest_line[3] - longest_line[1], longest_line[2] - longest_line[0]);

                    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
                    plane_normal.normalize();

                    // Find an arbitrary vector perpendicular to the plane normal
                    Eigen::Vector3f perp_vector = (std::abs(plane_normal.z()) < 0.99) 
                        ? Eigen::Vector3f(0, 0, 1).cross(plane_normal).normalized()
                        : Eigen::Vector3f(1, 0, 0).cross(plane_normal).normalized();

                    // Rotate perp_vector by `angle` around `plane_normal` to get x-axis
                    Eigen::AngleAxisf rotation(angle, plane_normal);
                    Eigen::Vector3f x_axis = rotation * perp_vector;  // Rotated vector in-plane

                    // Filter the x-axis direction to maintain consistency
                    if (filter_direction_.dot(x_axis) < 0) {
                        x_axis = -x_axis; // Flip to maintain consistency
                    }

                    // Update filter_direction_
                    filter_direction_ = x_axis;
                    
                    // Compute y-axis (perpendicular to both x and z)
                    Eigen::Vector3f y_axis = plane_normal.cross(x_axis).normalized();

                    // Construct rotation matrix
                    Eigen::Matrix3f rotation_matrix;
                    rotation_matrix.col(0) = x_axis;
                    rotation_matrix.col(1) = y_axis;
                    rotation_matrix.col(2) = plane_normal;  // z-axis is the plane normal

                    // Convert rotation matrix to quaternion
                    Eigen::Quaternionf quaternion(rotation_matrix);

                    if (std::isnan(quaternion.x()) || std::isnan(quaternion.y()) ||
                        std::isnan(quaternion.z()) || std::isnan(quaternion.w())) {
                        RCLCPP_WARN(this->get_logger(), "Invalid quaternion computed, skipping pose publishing.");
                        return;
                    }

                    geometry_msgs::msg::PoseStamped line_pose_msg;
                    line_pose_msg.header.stamp = color_image->header.stamp;
                    line_pose_msg.header.frame_id = "zed_left_camera_frame";

                    line_pose_msg.pose.position.x = centroid[0];
                    line_pose_msg.pose.position.y = centroid[1];
                    line_pose_msg.pose.position.z = centroid[2];

                    line_pose_msg.pose.orientation.x = quaternion.x();
                    line_pose_msg.pose.orientation.y = quaternion.y();
                    line_pose_msg.pose.orientation.z = quaternion.z();
                    line_pose_msg.pose.orientation.w = quaternion.w();

                    line_pose_pub_->publish(line_pose_msg);

                    RCLCPP_INFO(this->get_logger(), "Detected line published.");
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "No lines detected in bounding box.");
                }
                if (!cv_color_image.empty()) {
                    // Draw the bounding box on the color image
                    int x1 = std::max(center_x - width / 2, 0);
                    int y1 = std::max(center_y - height / 2, 0);
                    int x2 = std::min(center_x + width / 2, cv_color_image.cols - 1);
                    int y2 = std::min(center_y + height / 2, cv_color_image.rows - 1);
        
                    cv::rectangle(cv_color_image, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
        
                    // Draw the detected line on the color image
                    if (!lines.empty()) {
                        cv::Vec4i longest_line = lines[0];
                        double max_length = 0;
                    
                        // Find the longest line in the ROI
                        for (const auto &line : lines) {
                            double length = std::hypot(line[2] - line[0], line[3] - line[1]);
                            if (length > max_length) {
                                max_length = length;
                                longest_line = line;
                            }
                        }
                    
                        // Adjust line coordinates to the full image size using the same x1 ... as roi calculation
                        int x1 = std::max(center_x - width * line_detection_area_/10 , 0);
                        int y1 = std::max(center_y - height * line_detection_area_/10 , 0);
                        int x2 = std::min(center_x + width * line_detection_area_/10 , cv_color_image.cols - 1);
                        int y2 = std::min(center_y + height * line_detection_area_/10 , cv_color_image.rows - 1);
                        cv::Point pt1(longest_line[0] + x1, longest_line[1] + y1); // Add ROI offset
                        cv::Point pt2(longest_line[2] + x1, longest_line[3] + y1); // Add ROI offset
                    
                        // Draw the line on the full color image
                        cv::line(cv_color_image, pt1, pt2, cv::Scalar(0, 0, 255), 2);
                    
                        // Debugging output
                        RCLCPP_INFO(this->get_logger(), "ROI Coordinates: x1=%d, y1=%d, x2=%d, y2=%d", x1, y1, x2, y2);
                        RCLCPP_INFO(this->get_logger(), "Longest Line in ROI: (%d, %d) -> (%d, %d)", 
                                    longest_line[0], longest_line[1], longest_line[2], longest_line[3]);
                        RCLCPP_INFO(this->get_logger(), "Adjusted Line in Full Image: (%d, %d) -> (%d, %d)", 
                                    pt1.x, pt1.y, pt2.x, pt2.y);
                    }
        
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "No plane inliers found.");
            }
        }
    }
    // Convert the edited OpenCV image back to a ROS message
    sensor_msgs::msg::Image::SharedPtr edited_image_msg = cv_bridge::CvImage(color_image->header, "bgr8", cv_color_image).toImageMsg();

    // Publish the edited image
    processed_image_pub_->publish(*edited_image_msg);

    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "CvBridge Error: %s", e.what());
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ValveDetectionNode)