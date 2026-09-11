/**
 * @file gs130-run.c
 * @brief Smoke test script
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include "gs130_define.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define CAM_WINDOW 10
#define IMU_WINDOW 100
#define PRINT_US   20000

static volatile sig_atomic_t stop = 0;
static void on_sigint(int sig){(void)sig; stop = 1;}

static uint64_t now_us(void);
static double cam_fps(uint64_t ts);
static double imu_odr(uint64_t ts);
static void print_block(uint64_t cam_idx, uint64_t imu_idx, double fps, double odr,
                        uint64_t timestamp_left, uint64_t timestamp_right,
                        const gs130_imu_packet_t *imu);

int main(int argc, char **argv)
{
    if(argc < 8)return 1;
    signal(SIGINT, on_sigint);

    gs130_camera_mode_t mode = !strcmp(argv[3], "rect")   ? GS130_CAMERA_MODE_RECT :
                               !strcmp(argv[3], "resize") ? GS130_CAMERA_MODE_RESIZE :
                               GS130_CAMERA_MODE_RAW;

    gs130_config_t cfg = GS130_CONFIG(
        argv[1], argv[2], mode, atoi(argv[4]), atoi(argv[5]), atoi(argv[6]), atoi(argv[7]));

    /* Create Device Handle */
    gs130_device_t *dev = gs130_create();

    /* Init Device and Start DataFlow */
    int ret = 0;
    if(gs130_init(dev, &cfg) != GS130_OK || gs130_start(dev) != GS130_OK){
        fprintf(stderr, "init/start failed\n");
        ret = 1; goto out;
    }

    gs130_imu_packet_t imu_data = {0};
    double fps = 0, odr = 0;
    uint64_t timestamp_left = 0, timestamp_right = 0;
    uint64_t cam_idx = 0, imu_idx = 0;

    /* Get data */
    while(!stop){
        gs130_image_nv12_t left_image, right_image;
        while(gs130_get_nv12_frame(dev, &left_image, &right_image) == GS130_OK){
            fps = cam_fps(left_image.timestamp_ns);
            timestamp_left = left_image.timestamp_ns;
            timestamp_right = right_image.timestamp_ns;
            cam_idx++;
            free(left_image.data), free(right_image.data);   /* the buffer is ours to free */
        }
        gs130_imu_packet_t p;
        while(gs130_get_imu_packet(dev, &p) == GS130_OK){
            odr = imu_odr(p.timestamp_ns);
            imu_data = p;
            imu_idx++;
        }

        print_block(cam_idx, imu_idx, fps, odr, timestamp_left, timestamp_right, &imu_data);
        usleep(1000);
    }

out:
    gs130_stop(dev);
    gs130_deinit(dev);
    gs130_destroy(dev);
    return ret;
}

static double cam_fps(uint64_t ts)
{
    static uint64_t ring[CAM_WINDOW];
    static size_t pos, n;

    ring[pos] = ts;
    pos = (pos + 1) % CAM_WINDOW;
    if(++n < CAM_WINDOW)return 0.0;

    const uint64_t first = ring[pos], last = ring[(pos + CAM_WINDOW - 1) % CAM_WINDOW];
    return (last > first) ? (double)(CAM_WINDOW - 1) * 1e9 / (double)(last - first) : 0.0;
}

static double imu_odr(uint64_t ts)
{
    static uint64_t ring[IMU_WINDOW];
    static size_t pos, n;

    ring[pos] = ts;
    pos = (pos + 1) % IMU_WINDOW;
    if(++n < IMU_WINDOW)return 0.0;

    const uint64_t first = ring[pos], last = ring[(pos + IMU_WINDOW - 1) % IMU_WINDOW];
    return (last > first) ? (double)(IMU_WINDOW - 1) * 1e9 / (double)(last - first) : 0.0;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static void print_block(uint64_t cam_idx, uint64_t imu_idx, double fps, double odr,
                        uint64_t timestamp_left, uint64_t timestamp_right,
                        const gs130_imu_packet_t *imu)
{
    static uint64_t next_print = 0;
    const uint64_t now = now_us();
    if(now < next_print)return;
    next_print += PRINT_US;
    if(next_print <= now)next_print = now + PRINT_US;

    /* both rates need a full window, so they stay "--" until it has filled */
    char fps_s[16] = "    --", odr_s[16] = "    --";
    if(cam_idx >= CAM_WINDOW)snprintf(fps_s, sizeof(fps_s), "%6.2f", fps);
    if(imu_idx >= IMU_WINDOW)snprintf(odr_s, sizeof(odr_s), "%6.2f", odr);

    char line[2][288];
    if(cam_idx)snprintf(line[0], sizeof(line[0]),
            "Camera  [ID: %llu | fps: %s | Left timestamp: %.6f s | Right timestamp: %.6f s]",
            (unsigned long long)cam_idx - 1, fps_s, timestamp_left / 1e9, timestamp_right / 1e9);
    else snprintf(line[0], sizeof(line[0]), "Camera  [no frame yet]");

    if(imu_idx)snprintf(line[1], sizeof(line[1]),
            "IMU     [ID: %llu | odr: %s Hz | Timestamp: %.6f s | Accel %+7.2f %+7.2f %+7.2f m/s^2 | "
            "Gyro %+8.3f %+8.3f %+8.3f rad/s | Temp %+5.1f C]",
            (unsigned long long)imu_idx - 1, odr_s, imu->timestamp_ns / 1e9,
            imu->accel[0], imu->accel[1], imu->accel[2],
            imu->gyro[0], imu->gyro[1], imu->gyro[2], imu->temp);
    else snprintf(line[1], sizeof(line[1]), "IMU     [no packet yet]");

    const size_t width = strlen(line[0]) > strlen(line[1]) ? strlen(line[0]) : strlen(line[1]);
    for(size_t i = 0; i < width; i++)putchar('-');
    printf("\n%s\n%s\n", line[0], line[1]);
}
