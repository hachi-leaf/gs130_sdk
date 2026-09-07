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

// 双目立体校正：自动寻找最大 Map + 缩放系数
// 行为：
// - 输出网格尺寸：
//     - 使用源图尺寸（e.g.1088×1280）进行畸变矫正；
//     - 以源图尺寸为画幅，输出网格中心点对齐畸变矫正光心，自动搜素最大画幅；
//     - 针孔模型：输出与源尺寸相同的最大画幅网格（e.g.1088×1280）Map；
//     - 鱼眼模型：某一边方向（e.g.width）与去畸变蝴蝶形区域边界相切后，对另一边进行 32；
//     像素步进，直到这一边也碰边（e.g.h），输出扩张后的最大网格（e.g.1088x1312）。
// - 畸变矫正：
//     - 自动对齐左右图的 fx，fy
//     - 应用矫正 Map 后，图片光心 = 图片中心
//     - 进行矫正后的内外参修正，自动写回虚拟内外参到 *cal

Status stereo_rectify(StereoImuModel *cal,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t *grid_w, uint32_t *grid_h,
                      std::vector<RemapPoint> *left_map,
                      std::vector<RemapPoint> *right_map);

} // namespace base
} // namespace gs130

#endif // GS130_BASE_RECTIFY_HPP
