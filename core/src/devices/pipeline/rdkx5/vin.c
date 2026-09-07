/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * vin 节点：CIM + LPWM（立体同步 + IMU FSYNC）。
 */
#include "rdkx5.h"

#include <string.h>

int vin_open(hbn_vnode_handle_t *vin, int mipi_rx,
             uint32_t width, uint32_t height, uint32_t fps)
{
    const uint32_t period_us = 1000000U / fps;   /* LPWM 周期，单位 us */

    vin_node_attr_t node = {
        .cim_attr = {
            .mipi_rx = (uint32_t)mipi_rx,
            .vc_index = 0,
            .ipi_channel = 1,
            .cim_isp_flyby = 0,   /* VIN->ISP 走 offline(DDR)，多路必须如此 */
            .func = {
                .enable_frame_id = 1,
                .set_init_frame_id = 0,
                .hdr_mode = NOT_HDR,
                .time_stamp_en = 1,
                .time_stamp_mode = TS_IPI_VSYNC | TS_IPI_TRIGGER,
                .ts_src = (uint32_t)((mipi_rx == 0) ? 5 : 6),
            },
        },
        .lpwm_attr = {
            .enable = 1,
            .lpwm_chn_attr = {
                [0] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = 10, .duty_time = 1000, .threshold = 0, .adjust_step = 0 },
                [1] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = 10, .duty_time = 1000, .threshold = 0, .adjust_step = 0 },
                [2] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = 10, .duty_time = 1000, .threshold = 0, .adjust_step = 0 },
                [3] = { .trigger_source = 0, .trigger_mode = 0, .period = period_us,
                        .offset = 10, .duty_time = 1000, .threshold = 0, .adjust_step = 0 },
            },
        },
    };

    vin_ichn_attr_t ichn = {
        .width = width,
        .height = height,
        .format = 0x2B,      /* RAW10 */
    };

    vin_ochn_attr_t ochn = {
        .ddr_en = 1,
        .ochn_attr_type = VIN_BASIC_ATTR,
        .vin_basic_attr = {
            .format = 0x2B,
            .wstride = width * 2,   /* RAW10 每像素 2 字节 */
        },
    };

    if (hbn_vnode_open(HB_VIN, (uint32_t)mipi_rx, AUTO_ALLOC_ID, vin) != 0) {
        *vin = 0;
        return -1;
    }
    if (hbn_vnode_set_attr(*vin, &node) != 0 ||
        hbn_vnode_set_ichn_attr(*vin, 0, &ichn) != 0 ||
        hbn_vnode_set_ochn_attr(*vin, 0, &ochn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 3,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED,
    };
    if (hbn_vnode_set_ochn_buf_attr(*vin, 0, &alloc) != 0)
        return -1;
    return 0;
}
