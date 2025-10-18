# Valve Detection
[![Industrial CI](https://github.com/vortexntnu/valve-detection/actions/workflows/industrial-ci.yml/badge.svg)](https://github.com/vortexntnu/valve-detection/actions/workflows/industrial-ci.yml)
[![pre-commit.ci status](https://results.pre-commit.ci/badge/github/vortexntnu/valve-detection/main.svg)](https://results.pre-commit.ci/latest/github/vortexntnu/valve-detection/main)
[![codecov](https://codecov.io/github/vortexntnu/valve-detection/graph/badge.svg?token=6csTwaozlh)](https://codecov.io/github/vortexntnu/valve-detection)

---

### Overview

The **Valve Detection Node** estimates the **3D pose and orientation** of industrial valves from:

* A **depth image** or **point cloud**,
* An optional **color image**, and
* **2D detections** (e.g., YOLO bounding boxes).

It fits a **plane** to the valve annulus, computes the **3D intersection** of the valve center ray with that plane, and estimates the **rotation** of the valve handle using image-based line detection.

It can output:

* A `geometry_msgs::msg::PoseArray` of all valve poses,
* Debug point clouds (annulus points and segmented planes),
* Annotated color images showing detections, plane fits, and axes,
* Optional angle detection debug images.

---





## Launching the Node


```bash
ros2 launch valve_detection valve_detection.launch.py
```

## How it works

### 1. Input Synchronization

The node synchronizes:

* **Depth (or point cloud)**
* **Color image** (optional)
* **2D detections**

It supports multiple input modes via `use_depth_image` and `use_color_image`.

### 2. Annulus Extraction

For each bounding box, a ring-shaped region (annulus) around the center is extracted from the depth image or point cloud.

### 3. Plane Segmentation

RANSAC is used to fit a plane through the annulus points.

### 4. Ray–Plane Intersection

The center ray from the camera through the bounding box center is intersected with the plane to find the valve’s 3D position.

### 5. Orientation Estimation

* The **plane normal** defines one orientation axis.
* The **valve handle angle** (from image) defines the in-plane rotation.
* Intrinsics are used to back-project that angle into 3D.
* The result is a full 3×3 rotation matrix and quaternion.

### 6. Pose Publishing

A `PoseArray` message is published with one `Pose` per detected valve. If orientation computation failed no pose is published.

---

## Visualization Outputs

| Topic                    | Description                                 |                                  |
| ------------------------ | ------------------------------------------- | ---------------------------------------- |
| `/valve_detection_image` | Color image with bounding boxes and 3D axes | |
| `/valve_angle_image`     | Debug overlay showing detected Hough lines  |                       |
| `/bbx_annulus_pcl`       | Ring-shaped depth points used for plane fit                   |
| `/annulus_plane_pcl`     | Segmented plane points                      |

---


## Common Issues

| Issue                      | Possible Cause                                                               |
|----------------------------|-----------------------------------------------------------------------------|
| No poses published         | Missing plane segmentation (adjust `plane_ransac_threshold` or annulus size) |
| Angle NaN / no handle lines| Tune Hough and Canny thresholds                                              |


---

## Future work
* Use actual endpoints of line for backprojection to retrieve the perspective-correct plane angle. Now we just use rotation around optical axis.
TODO: For OBB, estimate line segment from BB size.
