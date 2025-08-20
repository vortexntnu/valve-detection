#ifndef TYPES_HPP
#define TYPES_HPP

namespace valve_detection {

struct CameraIntrinsics {
    double fx;
    double fy;
    double cx;
    double cy;
};

struct ImageDimensions {
    int width;
    int height;
};

struct ImageProperties {
    CameraIntrinsics intr;
    ImageDimensions dim;
};

struct BoundingBox {
    float center_x;
    float center_y;
    float size_x;
    float size_y;
    float theta; // Orientation in radians positive counter-clockwise
}

enum class AngleDetection {
    ENABLED,
    DISABLED
};

} // namespace valve_detection

#endif // TYPES_HPP