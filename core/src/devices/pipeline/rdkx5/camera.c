/**
 * @file camera.c
 * @brief camera node: fill the camera/mipi config and call hbn_camera_create.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#define _DEFAULT_SOURCE
#include "rdkx5.h"

#include <stdio.h>
#include <unistd.h>

/* sensor power: on=1 reset sequence 1->0->1 (30 ms per step), on=0 drive 0 */
int sensor_power(int gpio, int on)
{
    if(gpio < 0)return 0;

    char path[64];
    FILE *fp;

    snprintf(path, sizeof(path), "%d", gpio);
    fp = fopen("/sys/class/gpio/export", "w");
    if (fp) { fputs(path, fp); fclose(fp); }

    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    fp = fopen(path, "w");
    if (fp) { fputs("out", fp); fclose(fp); }

    for (const char *p = on ? "101" : "0"; *p; ++p) {
        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
        fp = fopen(path, "w");
        if (!fp)
            return -1;
        fputc(*p, fp);
        fclose(fp);
        if (on)
            usleep(30 * 1000);
    }
    return 0;
}

int camera_open(camera_handle_t *cam_fd, uint8_t i2c_addr,
                uint32_t width, uint32_t height, uint32_t fps,
                uint32_t line_length, uint32_t frame_length,
                uint16_t mipiclk, uint16_t settle, uint16_t mclk,
                const char *tuning_file)
{
    mipi_config_t mipi_cfg = {
        .rx_enable = 1,
        .rx_attr = {
            .phy = 0,
            .lane = 1,
            .datatype = 0x2B,        /* RAW10 */
            .fps = (uint16_t)fps,
            .mclk = mclk,
            .mipiclk = mipiclk,
            .width = (uint16_t)width,
            .height = (uint16_t)height,
            .linelenth = (uint16_t)line_length,
            .framelenth = (uint16_t)frame_length,
            .settle = settle,
            .channel_num = 1,
            .channel_sel = {0},
        },
    };

    camera_config_t cam_cfg = {
        .name = "sc132gs",
        .addr = i2c_addr,
        .sensor_mode = 6,            /* SLAVE: LPWM external trigger, both channels truly in sync */
        .fps = fps,
        .format = 0x2B,              /* RAW10 */
        .width = width,
        .height = height,
        .mipi_cfg = &mipi_cfg,
        .gpio_enable_bit = 0x01,
        .gpio_level_bit = 0x00,
    };
    snprintf(cam_cfg.calib_lname, sizeof(cam_cfg.calib_lname),
             "%s", tuning_file ? tuning_file : "disable");

    if (hbn_camera_create(&cam_cfg, cam_fd) != 0) {
        *cam_fd = 0;
        return -1;
    }
    return 0;
}
