/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * vse 节点：等比取景 + 缩放。
 */
#include "rdkx5.h"

#include <string.h>

int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h)
{
    if (in_w * out_h > out_w * in_h) {
        // 输入相对更宽：裁剪左右，roi.w = in_h * out_w / out_h
        return (in_h * out_w % out_h == 0) ? 0 : -1;
    }
    // 裁剪上下，roi.h = in_w * out_h / out_w
    return (in_w * out_h % out_w == 0) ? 0 : -1;
}

common_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                         uint32_t out_w, uint32_t out_h)
{
    common_rect_t roi;
    if (in_w * out_h > out_w * in_h) {
        // 输入相对更宽：裁剪左右
        roi.h = in_h;
        roi.w = in_h * out_w / out_h;
        roi.x = (in_w - roi.w) / 2;
        roi.y = 0;
    } else {
        // 裁剪上下
        roi.w = in_w;
        roi.h = in_w * out_h / out_w;
        roi.x = 0;
        roi.y = (in_h - roi.h) / 2;
    }
    return roi;
}

int vse_open(hbn_vnode_handle_t *vse, uint32_t in_w, uint32_t in_h,
             uint32_t out_w, uint32_t out_h, uint32_t vse_chn)
{
    const common_rect_t roi = aspect_roi(in_w, in_h, out_w, out_h);

    vse_attr_t attr = { 0 };

    vse_ichn_attr_t ichn = {
        .width = in_w, .height = in_h,
        .fmt = FRM_FMT_NV12, .bit_width = 8,
    };

    vse_ochn_attr_t ochn = {
        .chn_en = CAM_TRUE,
        .roi = { .x = roi.x, .y = roi.y, .w = roi.w, .h = roi.h },
        .target_w = out_w, .target_h = out_h,
        .fmt = FRM_FMT_NV12, .bit_width = 8,
    };

    if (hbn_vnode_open(HB_VSE, 0, AUTO_ALLOC_ID, vse) != 0) {
        *vse = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*vse, &attr) != 0 ||
        hbn_vnode_set_ichn_attr(*vse, 0, &ichn) != 0 ||
        hbn_vnode_set_ochn_attr(*vse, vse_chn, &ochn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 5,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*vse, vse_chn, &alloc) != 0)
        return -1;
    return 0;
}
