#include "valve_detection_ros/ros_utils.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "vortex_msgs/msg/landmark.hpp"

namespace valve_detection {

BoundingBox to_bbox(const vision_msgs::msg::BoundingBox2D& b) {
    BoundingBox o;
    o.center_x = static_cast<float>(b.center.position.x);
    o.center_y = static_cast<float>(b.center.position.y);
    o.size_x = static_cast<float>(b.size_x);
    o.size_y = static_cast<float>(b.size_y);
    o.theta = static_cast<float>(b.center.theta);  // radians
    return o;
}

geometry_msgs::msg::PoseArray make_pose_array(
    const std::vector<Pose>& poses,
    const std_msgs::msg::Header& header) {
    geometry_msgs::msg::PoseArray msg;
    msg.header = header;
    msg.poses.reserve(poses.size());
    for (const auto& p : poses) {
        geometry_msgs::msg::Pose po;
        po.position.x = p.position.x();
        po.position.y = p.position.y();
        po.position.z = p.position.z();
        po.orientation.x = p.orientation.x();
        po.orientation.y = p.orientation.y();
        po.orientation.z = p.orientation.z();
        po.orientation.w = p.orientation.w();
        msg.poses.push_back(po);
    }
    return msg;
}

vortex_msgs::msg::LandmarkArray make_landmark_array(
    const std::vector<Pose>& poses,
    const std_msgs::msg::Header& header,
    int type) {
    vortex_msgs::msg::LandmarkArray out;
    out.header = header;
    out.landmarks.reserve(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        vortex_msgs::msg::Landmark lm;
        lm.header = header;
        lm.id = static_cast<int32_t>(i);
        lm.type.value = type;
        lm.subtype.value = 0;  // unset — resolved by valve_subtype_resolver
        lm.pose.pose.position.x = poses[i].position.x();
        lm.pose.pose.position.y = poses[i].position.y();
        lm.pose.pose.position.z = poses[i].position.z();
        lm.pose.pose.orientation.x = poses[i].orientation.x();
        lm.pose.pose.orientation.y = poses[i].orientation.y();
        lm.pose.pose.orientation.z = poses[i].orientation.z();
        lm.pose.pose.orientation.w = poses[i].orientation.w();
        out.landmarks.push_back(lm);
    }
    return out;
}

cv::Mat decode_depth_to_float(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
    cv::Mat depth_img;
    // RealSense publishes depth as 16UC1 (uint16 millimetres).
    // cv_bridge type-casts without scaling, so we must divide by 1000.
    if (depth->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        depth->encoding == "16UC1") {
        cv_bridge::CvImageConstPtr cv_depth =
            cv_bridge::toCvShare(depth, "16UC1");
        cv_depth->image.convertTo(depth_img, CV_32FC1, 0.001);
    } else {
        cv_bridge::CvImageConstPtr cv_depth =
            cv_bridge::toCvShare(depth, "32FC1");
        depth_img = cv_depth->image.clone();
    }
    return depth_img;
}

}  // namespace valve_detection
