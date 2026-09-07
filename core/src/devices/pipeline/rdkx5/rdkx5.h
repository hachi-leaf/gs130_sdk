/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * RDK X5 平台骨架：节点接口。无共享上下文，每个节点直接收所需资源。
 */
#ifndef GS130_PIPELINE_RDKX5_H
#define GS130_PIPELINE_RDKX5_H

#include <stdint.h>
#include <stddef.h>

#include <hbn_api.h>
#include <hb_mem_mgr.h>
#include <cam_def.h>
#include <vin_cfg.h>
#include <isp_cfg.h>
#include <vse_cfg.h>
#include <gdc_cfg.h>
#include <hb_camera_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* sensor 供电：on=1 上电，on=0 下电 */
int sensor_power(int gpio, int on);

int camera_open(
    camera_handle_t *cam_fd, 
    uint8_t i2c_addr, 
    uint32_t width, uint32_t height, uint32_t fps,
    uint32_t line_length, uint32_t frame_length,
    uint16_t mipiclk, uint16_t settle, uint16_t mclk,
    const char *tuning_file);

int vin_open(
    hbn_vnode_handle_t *vin, 
    int mipi_rx, 
    uint32_t width, uint32_t height, uint32_t fps);

int isp_open(
    hbn_vnode_handle_t *isp, 
    uint32_t width, uint32_t height);

/* ROI 整除检查：ROI 宽高能被整除则 0，否则 -1（防整数除法截断） */
int roi_ratio_exact(uint32_t in_w, uint32_t in_h,
                    uint32_t out_w, uint32_t out_h);

/* 等比取景：从 in 裁出与 out 同宽高比的 ROI */
common_rect_t aspect_roi(uint32_t in_w, uint32_t in_h,
                         uint32_t out_w, uint32_t out_h);

int vse_open(
    hbn_vnode_handle_t *vse, 
    uint32_t in_w, uint32_t in_h,
    uint32_t out_w, uint32_t out_h, 
    uint32_t vse_chn);

int gdc_open(
    hbn_vnode_handle_t *gdc, 
    hb_mem_common_buf_t *gdc_bin,
    const void *map, uint32_t in_w, uint32_t in_h,
    uint32_t grid_w, uint32_t grid_h, int install_angle);

int vflow_build(
    hbn_vflow_handle_t *vflow, 
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, 
    hbn_vnode_handle_t isp,
    hbn_vnode_handle_t gdc, 
    hbn_vnode_handle_t vse, uint32_t vse_chn,
    hbn_vnode_handle_t *out_node, uint32_t *out_chn);

void teardown_cam(
    hbn_vflow_handle_t vflow, 
    camera_handle_t cam_fd,
    hbn_vnode_handle_t vin, hbn_vnode_handle_t isp,
    hbn_vnode_handle_t vse, 
    hbn_vnode_handle_t gdc,
    hb_mem_common_buf_t *gdc_bin, int reset_gpio);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GS130_PIPELINE_RDKX5_H */
