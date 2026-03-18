// valve_pose_ros.cpp
// ROS node: subscribes to depth, color, and detections; publishes valve poses and visualizations.
#include "valve_detection_ros/valve_pose_ros.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

namespace valve_detection {

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// Declares all ROS parameters, initializes the pose estimator, and sets up subscribers and publishers.
ValvePoseNode::ValvePoseNode(const rclcpp::NodeOptions& options)
: Node("valve_pose_node", options)
{
  const auto depth_topic      = declare_parameter<std::string>("depth_image_sub_topic");
  const auto color_topic      = declare_parameter<std::string>("color_image_sub_topic");
  const auto det_topic        = declare_parameter<std::string>("detections_sub_topic");
  const auto depth_info_topic = declare_parameter<std::string>("depth_image_info_topic");

  const auto pose_stamped_topic   = declare_parameter<std::string>("valve_pose_stamped_pub_topic");
  const auto pose_topic           = declare_parameter<std::string>("valve_poses_pub_topic");
  const auto lm_topic             = declare_parameter<std::string>("landmarks_pub_topic");
  const auto img_topic            = declare_parameter<std::string>("processed_image_pub_topic");
  const auto depth_color_topic    = declare_parameter<std::string>("depth_colormap_pub_topic");
  const auto valve_points_topic   = declare_parameter<std::string>("valve_points_pub_topic");
  const auto depth_cloud_topic    = declare_parameter<std::string>("depth_cloud_pub_topic");

  depth_colormap_vmin_ = static_cast<float>(declare_parameter<double>("depth_colormap_value_min"));
  depth_colormap_vmax_ = static_cast<float>(declare_parameter<double>("depth_colormap_value_max"));

  const int   yolo_w        = declare_parameter<int>("yolo_img_width");
  const int   yolo_h        = declare_parameter<int>("yolo_img_height");
  const float annulus_ratio = declare_parameter<float>("annulus_radius_ratio");
  const float ransac_thresh = declare_parameter<float>("plane_ransac_threshold");
  const int   ransac_iters  = declare_parameter<int>("plane_ransac_max_iterations");
  const float handle_off    = declare_parameter<float>("valve_handle_offset");

  use_color_image_       = declare_parameter<bool>("use_color_image");
  visualize_detections_  = declare_parameter<bool>("visualize_detections");
  debug_visualize_       = declare_parameter<bool>("debug_visualize");
  iou_duplicate_threshold_ = static_cast<float>(
      declare_parameter<double>("iou_duplicate_threshold"));
  output_frame_id_ = declare_parameter<std::string>("output_frame_id");

  landmark_type_    = declare_parameter<int>("landmark_type");
  landmark_subtype_ = declare_parameter<int>("landmark_subtype");

  detector_ = std::make_unique<PoseEstimator>(
      yolo_w, yolo_h, annulus_ratio, ransac_thresh, ransac_iters, handle_off);

  depth_color_extrinsic_ = d555_depth_to_color_extrinsic();
  detector_->set_depth_color_extrinsic(depth_color_extrinsic_);

  color_props_.intr.fx    = declare_parameter<double>("color_fx");
  color_props_.intr.fy    = declare_parameter<double>("color_fy");
  color_props_.intr.cx    = declare_parameter<double>("color_cx");
  color_props_.intr.cy    = declare_parameter<double>("color_cy");
  color_props_.dim.width  = declare_parameter<int>("color_image_width");
  color_props_.dim.height = declare_parameter<int>("color_image_height");

  detector_->set_color_image_properties(color_props_);
  detector_->calculate_letterbox_padding();

  cv_camera_matrix_ = (cv::Mat_<double>(3, 3)
      << color_props_.intr.fx, 0, color_props_.intr.cx,
         0, color_props_.intr.fy, color_props_.intr.cy,
         0, 0, 1);

  cv_dist_coeffs_ = (cv::Mat_<double>(5, 1)
      << declare_parameter<double>("color_d1"),
         declare_parameter<double>("color_d2"),
         declare_parameter<double>("color_d3"),
         declare_parameter<double>("color_d4"),
         declare_parameter<double>("color_d5"));

  got_info_ = true;
  RCLCPP_INFO(get_logger(),
      "Color intrinsics loaded from config (fx=%.2f fy=%.2f cx=%.2f cy=%.2f)",
      color_props_.intr.fx, color_props_.intr.fy,
      color_props_.intr.cx, color_props_.intr.cy);

  depth_props_.intr.fx    = declare_parameter<double>("depth_fx");
  depth_props_.intr.fy    = declare_parameter<double>("depth_fy");
  depth_props_.intr.cx    = declare_parameter<double>("depth_cx");
  depth_props_.intr.cy    = declare_parameter<double>("depth_cy");
  depth_props_.dim.width  = declare_parameter<int>("depth_image_width");
  depth_props_.dim.height = declare_parameter<int>("depth_image_height");

  detector_->set_depth_image_properties(depth_props_);
  got_depth_info_ = true;
  RCLCPP_INFO(get_logger(),
      "Depth intrinsics loaded from config (fx=%.2f fy=%.2f cx=%.2f cy=%.2f)",
      depth_props_.intr.fx, depth_props_.intr.fy,
      depth_props_.intr.cx, depth_props_.intr.cy);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1))
      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
      .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // Color camera_info is not subscribed — intrinsics and distortion are
  // always taken from config params above.
  depth_cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, info_qos,
      std::bind(&ValvePoseNode::depth_camera_info_cb, this, _1));

  pose_stamped_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_stamped_topic, qos);
  pose_pub_     = create_publisher<geometry_msgs::msg::PoseArray>(pose_topic, qos);
  landmark_pub_ = create_publisher<vortex_msgs::msg::LandmarkArray>(lm_topic, qos);
  image_pub_         = create_publisher<sensor_msgs::msg::Image>(img_topic, qos);
  depth_colormap_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_color_topic, qos);
  valve_points_pub_  = create_publisher<sensor_msgs::msg::PointCloud2>(valve_points_topic, qos);
  depth_cloud_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>(depth_cloud_topic, qos);

  if (debug_visualize_) {
    const auto ann_topic = declare_parameter<std::string>("debug_annulus_pub_topic");
    const auto pln_topic = declare_parameter<std::string>("debug_plane_pub_topic");
    annulus_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ann_topic, qos);
    plane_pub_   = create_publisher<sensor_msgs::msg::PointCloud2>(pln_topic, qos);
  }

  depth_sub_.subscribe(this, depth_topic, qos.get_rmw_qos_profile());
  color_sub_.subscribe(this, color_topic, qos.get_rmw_qos_profile());
  det_sub_.subscribe(this, det_topic, qos.get_rmw_qos_profile());

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), depth_sub_, color_sub_, det_sub_);
  sync_->registerCallback(
      std::bind(&ValvePoseNode::sync_cb, this, _1, _2, _3));
}

// One-shot callback that overrides depth intrinsics from the camera_info topic.
void ValvePoseNode::depth_camera_info_cb(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (got_depth_info_) return;

  depth_props_.intr.fx    = msg->k[0];
  depth_props_.intr.fy    = msg->k[4];
  depth_props_.intr.cx    = msg->k[2];
  depth_props_.intr.cy    = msg->k[5];
  depth_props_.dim.width  = static_cast<int>(msg->width);
  depth_props_.dim.height = static_cast<int>(msg->height);

  detector_->set_depth_image_properties(depth_props_);

  got_depth_info_ = true;
  depth_cam_info_sub_.reset();  // one-shot
  RCLCPP_INFO(get_logger(), "Depth camera_info received (fx=%.1f fy=%.1f)",
      depth_props_.intr.fx, depth_props_.intr.fy);
}

// Converts a ROS BoundingBox2D message to the internal BoundingBox struct.
BoundingBox ValvePoseNode::to_bbox(const vision_msgs::msg::BoundingBox2D& b) const
{
  BoundingBox o;
  o.center_x = static_cast<float>(b.center.position.x);
  o.center_y = static_cast<float>(b.center.position.y);
  o.size_x   = static_cast<float>(b.size_x);
  o.size_y   = static_cast<float>(b.size_y);
  o.theta    = static_cast<float>(b.center.theta);  // radians
  return o;
}

// Corrects the bbox center for lens distortion using the color camera matrix.
BoundingBox ValvePoseNode::undistort_bbox(const BoundingBox& bbox) const
{
  if (cv_camera_matrix_.empty() || cv_dist_coeffs_.empty()) return bbox;

  std::vector<cv::Point2f> pts{{bbox.center_x, bbox.center_y}};
  std::vector<cv::Point2f> undistorted;
  cv::undistortPoints(pts, undistorted,
      cv_camera_matrix_, cv_dist_coeffs_,
      cv::noArray(), cv_camera_matrix_);

  BoundingBox result = bbox;
  result.center_x = undistorted[0].x;
  result.center_y = undistorted[0].y;
  return result;
}

// Greedy NMS: sorts detections by score, keeps at most 2 non-overlapping boxes.
std::vector<size_t> ValvePoseNode::filter_duplicate_detections(
    const vision_msgs::msg::Detection2DArray& det) const
{
  const size_t n = det.detections.size();
  if (n == 0) return {};

  // Collect (score, index) pairs – use bbox area when no score is available.
  std::vector<std::pair<float, size_t>> scored;
  scored.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    float score = 0.0f;
    if (!det.detections[i].results.empty()) {
      score = static_cast<float>(
          det.detections[i].results[0].hypothesis.score);
    } else {
      const auto& b = det.detections[i].bbox;
      score = static_cast<float>(b.size_x * b.size_y);
    }
    scored.emplace_back(score, i);
  }
  std::sort(scored.begin(), scored.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  // Greedy NMS – keep at most 2.
  std::vector<size_t> kept;
  std::vector<bool>   suppressed(n, false);

  for (size_t si = 0; si < scored.size() && kept.size() < 2; ++si) {
    const size_t i = scored[si].second;
    if (suppressed[i]) continue;

    kept.push_back(i);

    const BoundingBox bi = to_bbox(det.detections[i].bbox);
    const float ai = bi.size_x * bi.size_y;
    const float bx1i = bi.center_x - bi.size_x * 0.5f;
    const float by1i = bi.center_y - bi.size_y * 0.5f;
    const float bx2i = bi.center_x + bi.size_x * 0.5f;
    const float by2i = bi.center_y + bi.size_y * 0.5f;

    for (size_t sj = si + 1; sj < scored.size(); ++sj) {
      const size_t j = scored[sj].second;
      if (suppressed[j]) continue;

      const BoundingBox bj = to_bbox(det.detections[j].bbox);
      const float aj = bj.size_x * bj.size_y;
      const float bx1j = bj.center_x - bj.size_x * 0.5f;
      const float by1j = bj.center_y - bj.size_y * 0.5f;
      const float bx2j = bj.center_x + bj.size_x * 0.5f;
      const float by2j = bj.center_y + bj.size_y * 0.5f;

      const float ix1 = std::max(bx1i, bx1j);
      const float iy1 = std::max(by1i, by1j);
      const float ix2 = std::min(bx2i, bx2j);
      const float iy2 = std::min(by2i, by2j);

      if (ix2 <= ix1 || iy2 <= iy1) continue;  // no overlap

      const float inter = (ix2 - ix1) * (iy2 - iy1);
      const float iou   = inter / (ai + aj - inter);
      const float iom   = inter / std::min(ai, aj);  // intersection over minimum

      if (iou > iou_duplicate_threshold_ || iom > 0.7f) {
        suppressed[j] = true;
      }
    }
  }

  return kept;
}

// Packs poses into a PoseArray message and publishes it.
void ValvePoseNode::publish_pose_array(const std::vector<Pose>& poses,
                                       const std_msgs::msg::Header& header)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header = header;
  msg.poses.reserve(poses.size());
  for (const auto& p : poses) {
    geometry_msgs::msg::Pose po;
    po.position.x    = p.position.x();
    po.position.y    = p.position.y();
    po.position.z    = p.position.z();
    po.orientation.x = p.orientation.x();
    po.orientation.y = p.orientation.y();
    po.orientation.z = p.orientation.z();
    po.orientation.w = p.orientation.w();
    msg.poses.push_back(po);
  }
  pose_pub_->publish(msg);
}

// Packs poses into a LandmarkArray message with type/subtype fields and publishes it.
void ValvePoseNode::publish_landmarks(const std::vector<Pose>& poses,
                                      const std_msgs::msg::Header& header)
{
  vortex_msgs::msg::LandmarkArray out;
  out.header = header;
  out.landmarks.reserve(poses.size());

  for (size_t i = 0; i < poses.size(); ++i) {
    vortex_msgs::msg::Landmark lm;
    lm.header        = header;
    lm.id            = static_cast<int32_t>(i);
    lm.type.value    = landmark_type_;
    lm.subtype.value = landmark_subtype_;

    lm.pose.pose.position.x    = poses[i].position.x();
    lm.pose.pose.position.y    = poses[i].position.y();
    lm.pose.pose.position.z    = poses[i].position.z();
    lm.pose.pose.orientation.x = poses[i].orientation.x();
    lm.pose.pose.orientation.y = poses[i].orientation.y();
    lm.pose.pose.orientation.z = poses[i].orientation.z();
    lm.pose.pose.orientation.w = poses[i].orientation.w();

    out.landmarks.push_back(lm);
  }
  landmark_pub_->publish(out);
}

// Main synchronized callback: runs NMS, estimates poses, and publishes all outputs.
void ValvePoseNode::sync_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const sensor_msgs::msg::Image::ConstSharedPtr& color,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& det)
{
  if (!got_info_) return;
  if (!depth || !det) return;

  cv::Mat depth_color;
  const bool publish_colormap = depth_colormap_pub_->get_subscription_count() > 0;
  if (publish_colormap) {
    cv_bridge::CvImageConstPtr cv_raw = cv_bridge::toCvShare(depth, "16UC1");
    cv::Mat depth_f;
    cv_raw->image.convertTo(depth_f, CV_32FC1);
    const float scale = 255.0f / (depth_colormap_vmax_ - depth_colormap_vmin_);
    cv::Mat depth_8u;
    depth_f.convertTo(depth_8u, CV_8UC1, scale, -depth_colormap_vmin_ * scale);
    cv::applyColorMap(depth_8u, depth_color, cv::COLORMAP_TURBO);
  }

  cv::Mat color_vis;
  const bool publish_image = color && image_pub_->get_subscription_count() > 0;
  if (publish_image) {
    cv_bridge::CvImageConstPtr cv_color = cv_bridge::toCvShare(color, "bgr8");
    color_vis = cv_color->image.clone();
  }

  if (det->detections.empty()) {
    publish_pose_array({}, depth->header);
    publish_landmarks({}, depth->header);
    if (publish_colormap) {
      depth_colormap_pub_->publish(
          *cv_bridge::CvImage(depth->header, "bgr8", depth_color).toImageMsg());
    }
    if (publish_image) {
      image_pub_->publish(
          *cv_bridge::CvImage(color->header, "bgr8", color_vis).toImageMsg());
    }
    return;
  }

  const std::vector<size_t> kept = filter_duplicate_detections(*det);

  // RealSense publishes depth as 16UC1 (uint16 millimetres).
  // cv_bridge type-casts without scaling, so we must divide by 1000.
  cv::Mat depth_img;
  if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      depth->encoding == "16UC1") {
    cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth, "16UC1");
    cv_depth->image.convertTo(depth_img, CV_32FC1, 0.001);
  } else {
    cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth, "32FC1");
    depth_img = cv_depth->image.clone();
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ann_dbg(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pln_dbg(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<BoundingBox> kept_boxes;
  std::vector<BoundingBox> raw_boxes;   // raw detection coords for depth colormap
  std::vector<Pose>        poses;

  for (size_t idx : kept) {
    // Parse bounding box (YOLO-space), map to original image, then undistort.
    BoundingBox yolo_box = to_bbox(det->detections[idx].bbox);
    BoundingBox org_box  = yolo_box;
    org_box              = undistort_bbox(org_box);

    raw_boxes.push_back(yolo_box);
    kept_boxes.push_back(org_box);

    Pose pose;
    if (detector_->compute_pose_from_depth(
            depth_img, yolo_box, pose, ann_dbg, pln_dbg, true)) {
      poses.push_back(pose);
    }
  }

  if (depth_cloud_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*ann_dbg, cloud_msg);
    std_msgs::msg::Header cloud_header = depth->header;
    if (!output_frame_id_.empty()) cloud_header.frame_id = output_frame_id_;
    cloud_msg.header = cloud_header;
    depth_cloud_pub_->publish(cloud_msg);
  }

  if (publish_image) {
    for (size_t i = 0; i < kept_boxes.size(); ++i) {
      const auto& box = kept_boxes[i];
      const float angle_deg = box.theta * 180.0f / static_cast<float>(M_PI);
      cv::RotatedRect rrect(
          cv::Point2f(box.center_x, box.center_y),
          cv::Size2f(box.size_x, box.size_y),
          angle_deg);
      cv::Point2f corners[4];
      rrect.points(corners);
      for (int j = 0; j < 4; ++j) {
        cv::line(color_vis, corners[j], corners[(j + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      }
    }
    image_pub_->publish(
        *cv_bridge::CvImage(color->header, "bgr8", color_vis).toImageMsg());
  }

  if (publish_colormap) {
    for (size_t i = 0; i < raw_boxes.size(); ++i) {
      const auto& box = raw_boxes[i];

      // Project each OBB corner from color image space to depth image space
      // using the full intrinsic + extrinsic pipeline.
      // Fall back to the raw color pixel when no valid pose depth is available.
      const float Z = (i < poses.size() && poses[i].position.z() > 0.0f)
                      ? poses[i].position.z() : 0.0f;

      const float angle_deg = box.theta * 180.0f / static_cast<float>(M_PI);
      cv::RotatedRect rrect(
          cv::Point2f(box.center_x, box.center_y),
          cv::Size2f(box.size_x, box.size_y),
          angle_deg);
      cv::Point2f corners[4];
      rrect.points(corners);

      if (Z > 0.0f && got_depth_info_) {
        for (auto& c : corners) {
          c = project_color_pixel_to_depth(
                c.x, c.y, Z, color_props_, depth_props_, depth_color_extrinsic_);
        }
      }

      for (int j = 0; j < 4; ++j) {
        cv::line(depth_color, corners[j], corners[(j + 1) % 4],
                 cv::Scalar(0, 255, 0), 2);
      }
    }
    depth_colormap_pub_->publish(
        *cv_bridge::CvImage(depth->header, "bgr8", depth_color).toImageMsg());
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
  if (!output_frame_id_.empty()) pose_header.frame_id = output_frame_id_;

  publish_pose_array(poses, pose_header);
  publish_landmarks(poses, pose_header);

  // Publish valve center positions as a PointCloud2 for Foxglove 3D view.
  if (valve_points_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZ> pts;
    for (const auto& p : poses) {
      pts.points.push_back({p.position.x(), p.position.y(), p.position.z()});
    }
    pts.width  = static_cast<uint32_t>(pts.points.size());
    pts.height = 1;
    pts.is_dense = true;
    sensor_msgs::msg::PointCloud2 pc_msg;
    pcl::toROSMsg(pts, pc_msg);
    pc_msg.header = pose_header;
    valve_points_pub_->publish(pc_msg);
  }

  // Publish best (first) detection as PoseStamped for easy Foxglove 3D view.
  if (!poses.empty()) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = pose_header;
    ps.pose.position.x    = poses[0].position.x();
    ps.pose.position.y    = poses[0].position.y();
    ps.pose.position.z    = poses[0].position.z();
    ps.pose.orientation.x = poses[0].orientation.x();
    ps.pose.orientation.y = poses[0].orientation.y();
    ps.pose.orientation.z = poses[0].orientation.z();
    ps.pose.orientation.w = poses[0].orientation.w();
    pose_stamped_pub_->publish(ps);
  }
}

}  // namespace valve_detection

RCLCPP_COMPONENTS_REGISTER_NODE(valve_detection::ValvePoseNode)
