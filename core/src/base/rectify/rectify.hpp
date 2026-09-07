/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * Binocular stereo correction
 */
#ifndef GS130_BASE_RECTIFY_HPP
#define GS130_BASE_RECTIFY_HPP

#include <cstdint>
#include <vector>

#include "types.hpp"

namespace gs130 {
namespace base {

// Binocular stereo rectification: automatically find the maximum Map + scale factor
// Behavior:
// - Output grid size:
//     - Undistort using the source image size (e.g. 1088x1280);
//     - Use the source size as the frame, align output grid centers to the undistorted optical center, and auto-search the maximum frame;
//     - Pinhole model: output a maximum-frame grid Map at the source size (e.g. 1088x1280);
//     - Fisheye model: once one side (e.g. width) is tangent to the undistorted butterfly-region boundary, grow the other side in 32-pixel
//       steps until that side also touches the border (e.g. h); output the expanded maximum grid (e.g. 1088x1312).
// - Distortion correction:
//     - Automatically align the left/right images' fx, fy
//     - After applying the rectification Map, image optical center = image center
//     - Correct the post-rectification intrinsics/extrinsics and automatically write the virtual intrinsics/extrinsics back to *cal

Status stereo_rectify(StereoImuModel *cal,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t *grid_w, uint32_t *grid_h,
                      std::vector<RemapPoint> *left_map,
                      std::vector<RemapPoint> *right_map);

} // namespace base
} // namespace gs130

#endif // GS130_BASE_RECTIFY_HPP
