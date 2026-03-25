#pragma once

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>

#include "valve_detection/types.hpp"
#include "vortex_msgs/msg/landmark_array.hpp"

#include <vector>

namespace valve_detection {

// Converts a ROS BoundingBox2D message to the internal BoundingBox type.
BoundingBox to_bbox(const vision_msgs::msg::BoundingBox2D& b);

// Builds a PoseArray message from a list of poses.
geometry_msgs::msg::PoseArray make_pose_array(
    const std::vector<Pose>& poses,
    const std_msgs::msg::Header& header);

// Builds a LandmarkArray message from a list of poses.
// Subtype is always 0 (unset); resolved downstream by valve_subtype_resolver.
vortex_msgs::msg::LandmarkArray make_landmark_array(
    const std::vector<Pose>& poses,
    const std_msgs::msg::Header& header,
    int type);

// Decodes a ROS depth image to a CV_32FC1 mat in metres.
cv::Mat decode_depth_to_float(
    const sensor_msgs::msg::Image::ConstSharedPtr& depth);

}  // namespace valve_detection
