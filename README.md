# Valve Detection
[![Industrial CI](https://github.com/vortexntnu/valve-detection/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/valve-detection/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/valve-detection/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/valve-detection/main)
[![codecov](https://codecov.io/github/vortexntnu/valve-detection/graph/badge.svg?token=6csTwaozlh)](https://codecov.io/github/vortexntnu/valve-detection)

---

## Overview

The **Valve Detection Node** estimates the **3D pose and orientation** of industrial valves from a synchronized depth image, color image, and 2D oriented bounding box detections (e.g., from a YOLO OBB model).

For each detection it:
1. Extracts a point cloud from the depth image aligned to the color camera frame.
2. Fits a plane to those points using RANSAC.
3. Intersects the viewing ray with the plane to find the 3D position.
4. Derives the orientation from the plane normal and the bounding box angle.

---

## File Layout

| File | Responsibility |
|------|---------------|
| `valve_pose_ros.hpp/.cpp` | ROS node — subscriptions, publishing, NMS, camera data ownership |
| `ros_utils.hpp/.cpp` | ROS message conversions — `to_bbox`, `make_pose_array`, `make_landmark_array`, `decode_depth_to_float` |
| `depth_image_processing.hpp/.cpp` | Depth transforms — back-projection, point cloud extraction, depth-to-color pixel mapping |
| `pose_estimator.hpp/.cpp` | Normal/plane estimation — RANSAC plane fit, ray–plane intersection, rotation matrix, pose |
| `types.hpp` | Shared data types (`BoundingBox`, `Pose`, `ImageProperties`, `DepthColorExtrinsic`) |

---

## Launching the Node

```bash
ros2 launch valve_detection valve_detection.launch.py
```

### With debug visualization

```bash
ros2 launch valve_detection valve_detection.launch.py debug_visualize:=true
```

This enables the additional debug topics listed below (`/valve_poses`, `/valve_detection_depth_colormap`, `/valve_depth_cloud`, `/bbx_annulus_pcl`, `/annulus_plane_pcl`).

---

## How It Works

### 1. Input Synchronization

The node subscribes to three topics using `message_filters::ApproximateTime`:

| Topic (default) | Type |
|---|---|
| `/realsense/D555_409122300281_Depth` | `sensor_msgs/Image` (16UC1 mm) |
| `/realsense/D555_409122300281_Color` | `sensor_msgs/Image` (BGR8) |
| `/yolo_obb_object_detection/detections` | `vision_msgs/Detection2DArray` |

### 2. Duplicate Suppression (NMS)

Detections are filtered with greedy NMS. Two boxes are considered duplicates when their IoU or intersection-over-minimum exceeds the configured threshold. At most 2 detections are kept per frame.

### 3. Point Cloud Extraction

For each kept bounding box, all depth pixels whose reprojection into the color frame falls inside the oriented bounding box are back-projected to 3D (in the color camera frame). The depth-to-color extrinsic is applied to correctly handle the baseline offset between the two cameras.

### 4. Plane Segmentation

RANSAC fits a plane through the extracted point cloud.

### 5. Ray–Plane Intersection

The viewing ray through the bounding box center (using color intrinsics) is intersected with the fitted plane to find the valve's 3D position. The position is optionally shifted along the plane normal by `valve_handle_offset` to account for the valve handle protrusion.

### 6. Orientation Estimation

The plane normal defines the Z-axis. The bounding box angle is back-projected onto the plane using the color intrinsics to determine the in-plane X-axis, giving a full 3×3 rotation matrix converted to a quaternion.

### 7. Depth Colormap Visualization

The bounding box is reprojected from color image space to depth image space using the full intrinsic + extrinsic pipeline (`project_color_pixel_to_depth`) so the overlay is correctly aligned on the depth colormap.

---

## Published Topics

| Topic | Type | Always | Description |
|-------|------|--------|-------------|
| `/valve_landmarks` | `vortex_msgs/LandmarkArray` | Yes | All detection poses with landmark type/subtype |
| `/valve_poses` | `geometry_msgs/PoseArray` | Debug | All detection poses |
| `/valve_detection_depth_colormap` | `sensor_msgs/Image` | Debug | Depth colormap with OBB overlays |
| `/valve_depth_cloud` | `sensor_msgs/PointCloud2` | Debug | Points used for plane fit |
| `/bbx_annulus_pcl` | `sensor_msgs/PointCloud2` | Debug | Extracted annulus points |
| `/annulus_plane_pcl` | `sensor_msgs/PointCloud2` | Debug | RANSAC plane inliers |

Debug topics are only published when `debug_visualize:=true`.

---

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `annulus_radius_ratio` | `0.8` | Inner radius of the extraction ring as fraction of outer radius |
| `plane_ransac_threshold` | `0.01` | RANSAC inlier distance threshold (m) |
| `plane_ransac_max_iterations` | `50` | RANSAC iteration limit |
| `valve_handle_offset` | `0.05` | Shift along plane normal to reach handle (m) |
| `iou_duplicate_threshold` | `0.5` | IoU threshold for NMS |
| `yolo_img_width/height` | `640` | YOLO letterbox reference size for bbox remapping |
| `debug_visualize` | `false` | Enable debug visualization topics |
| `output_frame_id` | `camera_color_optical_frame` | TF frame for published poses |
| `depth_to_color_tx/ty/tz` | `-0.059, 0, 0` | Depth-to-color extrinsic translation (m) |

Camera intrinsics (`color_fx/fy/cx/cy`, `depth_fx/fy/cx/cy`) and distortion coefficients are set in `config/valve_detection_params.yaml`. Both color and depth intrinsics can alternatively be received from `CameraInfo` topics and will override the config values.

---

## Common Issues

| Issue | Possible Cause |
|-------|---------------|
| No poses published | Too few plane inliers — lower `plane_ransac_threshold` or increase `annulus_radius_ratio` |
| Pose position offset | Wrong `valve_handle_offset` or incorrect camera intrinsics/extrinsic |
| OBB misaligned on depth colormap | Incorrect depth-to-color extrinsic — check `depth_to_color_tx/ty/tz` in the config |
| Duplicate poses | Lower `iou_duplicate_threshold` |

---

## Future Work

- Use actual OBB edge endpoints for back-projection to get the perspective-correct in-plane angle instead of rotating around the optical axis.
