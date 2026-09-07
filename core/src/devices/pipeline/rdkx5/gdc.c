/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * gdc node: apply +0.5/clamp/rotation to the map, encode it as a bin, then open the GDC vnode.
 * map is a RemapPoint array (layout matches point_t, guaranteed by static_assert).
 */
#include "rdkx5.h"

#include <stdlib.h>
#include <string.h>

int gdc_open(hbn_vnode_handle_t *gdc, hb_mem_common_buf_t *gdc_bin,
             const void *map, uint32_t in_w, uint32_t in_h,
             uint32_t grid_w, uint32_t grid_h, int install_angle)
{
    const uint32_t npts = grid_w * grid_h;
    const point_t *m = (const point_t *)map;

    // the rotation target (GDC input) is portrait: xmax/ymax always use portrait dimensions
    const double xmax = (double)in_w - 1.0;
    const double ymax = (double)in_h - 1.0;
    // the map's source coords swap width/height in landscape; the clamp bounds follow the swap
    const int    swap     = (install_angle == 90 || install_angle == 270);
    const double src_xmax = swap ? ymax : xmax;
    const double src_ymax = swap ? xmax : ymax;

    point_t *pts = (point_t *)malloc((size_t)npts * sizeof(point_t));
    if (!pts)
        return -1;

    for (uint32_t i = 0; i < npts; ++i) {
        /* GDC bilinear-interpolation compensation: +0.5 sub-pixel, clamp to avoid black edges */
        double x = m[i].x + 0.5;
        double y = m[i].y + 0.5;
        if (x > src_xmax) x = src_xmax;
        if (y > src_ymax) y = src_ymax;
        /* the install rotation applies to the source coords, converting landscape sampling back to portrait */
        switch (install_angle) {
        case 90:  pts[i].x = y;        pts[i].y = ymax - x; break;
        case 180: pts[i].x = xmax - x; pts[i].y = ymax - y; break;
        case 270: pts[i].x = xmax - y; pts[i].y = x;        break;
        default:  pts[i].x = x;        pts[i].y = y;        break;
        }
    }

    param_t param = {
        .format = FMT_SEMIPLANAR_420,
        .in = { .w = in_w, .h = in_h },
        .out = { .w = grid_w, .h = grid_h },
        .fov = 180.0,
        .diameter = in_h,
    };

    window_t win = {
        .transform = CUSTOM,
        .strength = 1.0,
        .strengthY = 1.0,
        .keep_ratio = 1,
        .FOV_h = 90.0,
        .FOV_w = 90.0,
        .trapezoid_left_angle = 90.0,
        .trapezoid_right_angle = 90.0,
        .out_r = { .w = grid_w, .h = grid_h },
        .input_roi_r = { .w = in_w, .h = in_h },
        .zoom = 1.0,
        .custom = {
            .full_tile_calc = 1,
            .tile_incr_x = 50,
            .tile_incr_y = 50,
            .w = (int32_t)grid_w - 1,
            .h = (int32_t)grid_h - 1,
            .centerx = (double)(grid_w / 2),
            .centery = (double)(grid_h / 2),
            .points = pts,
        },
    };

    uint32_t *cfg_buf  = NULL;
    uint64_t  cfg_size = 0;
    if (hbn_gen_gdc_bin(&param, &win, 1, &cfg_buf, &cfg_size) != 0 ||
        cfg_buf == NULL || cfg_size == 0) {
        free(cfg_buf);
        free(pts);
        return -1;
    }

    const int64_t flags = HB_MEM_USAGE_MAP_INITIALIZED |
                          HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
                          HB_MEM_USAGE_CPU_READ_OFTEN |
                          HB_MEM_USAGE_CPU_WRITE_OFTEN |
                          HB_MEM_USAGE_CACHED;
    memset(gdc_bin, 0, sizeof(*gdc_bin));
    if (hb_mem_alloc_com_buf(cfg_size, flags, gdc_bin) != 0 ||
        gdc_bin->virt_addr == NULL) {
        free(cfg_buf);
        free(pts);
        return -1;
    }
    memcpy(gdc_bin->virt_addr, cfg_buf, (size_t)cfg_size);
    free(cfg_buf);
    free(pts);

    if (hb_mem_flush_buf(gdc_bin->fd, 0, cfg_size) != 0)
        return -1;

    if (hbn_vnode_open(HB_GDC, 0, AUTO_ALLOC_ID, gdc) != 0) {
        *gdc = 0;
        return -1;
    }

    gdc_attr_t attr = {
        .config_addr   = gdc_bin->phys_addr,
        .config_size   = (uint32_t)gdc_bin->size,
        .binary_ion_id = gdc_bin->share_id,
        .binary_offset = gdc_bin->offset,
        .total_planes  = 2,
    };
    if (hbn_vnode_set_attr(*gdc, &attr) != 0)
        return -1;

    gdc_ichn_attr_t ichn = {
        .input_width  = in_w,
        .input_height = in_h,
        .input_stride = in_w,
    };
    if (hbn_vnode_set_ichn_attr(*gdc, 0, &ichn) != 0)
        return -1;

    gdc_ochn_attr_t ochn = {
        .output_width  = grid_w,
        .output_height = grid_h,
        .output_stride = grid_w,
    };
    if (hbn_vnode_set_ochn_attr(*gdc, 0, &ochn) != 0)
        return -1;

    hbn_buf_alloc_attr_t alloc = {
        .buffers_num = 5,
        .is_contig = 1,
        .flags = HB_MEM_USAGE_CPU_READ_OFTEN |
                 HB_MEM_USAGE_CPU_WRITE_OFTEN |
                 HB_MEM_USAGE_CACHED |
                 HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF,
    };
    if (hbn_vnode_set_ochn_buf_attr(*gdc, 0, &alloc) != 0)
        return -1;
    return 0;
}
