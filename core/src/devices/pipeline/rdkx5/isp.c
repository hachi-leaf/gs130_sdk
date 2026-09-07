/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * isp node: offline (DDR) ISP, NV12 output.
 */
#include "rdkx5.h"

#include <string.h>

int isp_open(hbn_vnode_handle_t *isp, uint32_t width, uint32_t height)
{
    isp_attr_t attr = {
        .input_mode = DDR_MODE,      /* offline; multi-channel ISP must run offline */
        .sensor_mode = ISP_NORMAL_M,
        .crop = { .x = 0, .y = 0, .w = width, .h = height },
    };

    isp_ichn_attr_t ichn = {
        .width = width,
        .height = height,
        .fmt = FRM_FMT_RAW,
        .bit_width = 10,
    };

    isp_ochn_attr_t ochn = {
        .ddr_en = CAM_TRUE,
        .fmt = FRM_FMT_NV12,
        .bit_width = 8,
    };

    if (hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, isp) != 0) {
        *isp = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*isp, &attr) != 0 ||
        hbn_vnode_set_ochn_attr(*isp, ISP_MAIN_FRAME, &ochn) != 0 ||
        hbn_vnode_set_ichn_attr(*isp, 0, &ichn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 5,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*isp, ISP_MAIN_FRAME, &alloc) != 0)
        return -1;
    return 0;
}
