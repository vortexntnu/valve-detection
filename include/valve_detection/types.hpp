#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace valve_detection {

struct CameraIntrinsics {
    double fx{0}, fy{0}, cx{0}, cy{0};
};

struct ImageDimensions {
    int width{0}, height{0};
};

struct ImageProperties {
    CameraIntrinsics intr;
    ImageDimensions dim;
};

struct BoundingBox {
    float center_x{0};
    float center_y{0};
    float size_x{0};
    float size_y{0};
    float theta{0};  // radians
};

struct Pose {
    Eigen::Vector3f position{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf orientation{Eigen::Quaternionf::Identity()};
};

// Rigid transform from depth camera frame to color camera frame.
// Rotation R and translation t satisfy:  P_color = R * P_depth + t
struct DepthColorExtrinsic {
    Eigen::Matrix3f R{Eigen::Matrix3f::Identity()};
    Eigen::Vector3f t{Eigen::Vector3f::Zero()};
};

}  // namespace valve_detection
