// valve_subtype_resolver.cpp
// Subscribes to valve landmarks (subtype unset) published by valve_detection.
// For each landmark, looks up the full transform from the landmark frame to the
// world frame via TF2, rotates the valve plane normal into the world frame, and
// classifies the valve as VALVE_VERTICAL or VALVE_HORIZONTAL based on how much
// the normal deviates from the world Z-axis.  Republishes with subtype set.
#include "valve_subtype_resolver/valve_subtype_resolver.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex_msgs/msg/landmark_subtype.hpp>

namespace valve_subtype_resolver {

using std::placeholders::_1;

ValveSubtypeResolverNode::ValveSubtypeResolverNode(
    const rclcpp::NodeOptions& options)
    : Node("valve_subtype_resolver_node", options) {
    const std::string drone = declare_parameter<std::string>("drone", "moby");
    const std::string odom_frame_base =
        declare_parameter<std::string>("odom_frame", "odom");
    world_frame_ = drone + "/" + odom_frame_base;
    vertical_threshold_ = declare_parameter<double>("vertical_threshold", 0.5);

    const auto landmarks_in = declare_parameter<std::string>(
        "landmarks_sub_topic", "/valve_landmarks");
    const auto landmarks_out = declare_parameter<std::string>(
        "landmarks_pub_topic", "/valve_landmarks_typed");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    landmark_sub_ = create_subscription<vortex_msgs::msg::LandmarkArray>(
        landmarks_in, qos,
        std::bind(&ValveSubtypeResolverNode::landmarks_cb, this, _1));

    landmark_pub_ =
        create_publisher<vortex_msgs::msg::LandmarkArray>(landmarks_out, qos);
}

void ValveSubtypeResolverNode::landmarks_cb(
    const vortex_msgs::msg::LandmarkArray::SharedPtr msg) {
    vortex_msgs::msg::LandmarkArray out = *msg;
    for (auto& lm : out.landmarks)
        lm.subtype.value = static_cast<uint16_t>(resolve_subtype(lm));
    landmark_pub_->publish(out);
}

// Resolves the valve subtype by rotating the valve plane normal into the world
// frame and checking its Z component.
//
// The valve plane normal is the Z-axis of the landmark orientation quaternion.
// It is transformed from the landmark frame into the world frame via a TF2
// lookup that traverses the full chain:
//   <drone>/front_camera_depth_optical → <drone>/front_camera_depth
//   → <drone>/front_camera_link → <drone>/base_link → <drone>/odom
//
// Classification:
//   |world_nz| >= vertical_threshold → VALVE_VERTICAL (1)
//     The normal points mostly up or down; the valve face is horizontal,
//     meaning the valve wheel spins around a vertical axis.
//   |world_nz| < vertical_threshold → VALVE_HORIZONTAL (2)
//     The normal is mostly in the XY plane; the valve face is vertical,
//     meaning the valve wheel spins around a horizontal axis.
//
// Returns 0 (unknown) if the TF lookup fails.
int ValveSubtypeResolverNode::resolve_subtype(
    const vortex_msgs::msg::Landmark& landmark) {
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(
            world_frame_, landmark.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "TF lookup failed: %s", ex.what());
        return 0;
    }

    // Extract the valve plane normal (Z-axis of the landmark orientation).
    // Rotating [0,0,1] by quaternion q gives:
    //   nx = 2(qx*qz + qw*qy)
    //   ny = 2(qy*qz - qw*qx)
    //   nz = 1 - 2(qx^2 + qy^2)
    const auto& lq = landmark.pose.pose.orientation;
    geometry_msgs::msg::Vector3Stamped normal_cam;
    normal_cam.header = landmark.header;
    normal_cam.vector.x = 2.0 * (lq.x * lq.z + lq.w * lq.y);
    normal_cam.vector.y = 2.0 * (lq.y * lq.z - lq.w * lq.x);
    normal_cam.vector.z = 1.0 - 2.0 * (lq.x * lq.x + lq.y * lq.y);

    // Rotate the normal into the world frame.
    // tf2::doTransform on a Vector3Stamped applies only the rotation.
    geometry_msgs::msg::Vector3Stamped normal_world;
    tf2::doTransform(normal_cam, normal_world, transform);

    using ST = vortex_msgs::msg::LandmarkSubtype;
    if (std::abs(normal_world.vector.z) >= vertical_threshold_)
        return ST::VALVE_VERTICAL;
    return ST::VALVE_HORIZONTAL;
}

}  // namespace valve_subtype_resolver

RCLCPP_COMPONENTS_REGISTER_NODE(
    valve_subtype_resolver::ValveSubtypeResolverNode)
