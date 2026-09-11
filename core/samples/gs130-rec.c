/**
 * @file gs130-rec.c
 * @brief Record script
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 *
 * usage: gs130-rec <cam_from> <cam_to> <imu_from> <imu_to> <out_dir> <stitch>
 *                  <platform> <device> <mode> <w> <h> <fps> <odr>
 *
 *   <cam|imu>_from/_to  half-open index range to keep, from = -1 for none
 *   out_dir             created when missing
 *   stitch              1 records one left-right frame per index, 0 one file per eye
 *
 * output: <out_dir>/cam/%06ld[_L|_R].nv12 and <out_dir>/imu.csv
 */
#include "gs130.h"
#include "gs130_define.h"

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define MAX_FRAMES 512   /* the kept range lives in RAM: ~2 GiB at 1088x1280 stereo */
#define REPORT_US  1000000

static volatile sig_atomic_t stop = 0;
static void on_sigint(int sig){(void)sig; stop = 1;}

static uint64_t now_us(void);
static void mkdir_p(const char *path);
static void report(uint64_t t0, bool keep_cam, size_t ncam, size_t cam_cap,
                   bool keep_imu, size_t nimu, size_t imu_cap);
static bool get_frame(gs130_device_t *dev, bool stitch, gs130_image_nv12_t f[2]);
static void save_frame(const char *dir, long index, const char *suffix,
                       const gs130_image_nv12_t *image);
static void save_imu(const char *dir, long from, const gs130_imu_packet_t *imu, size_t n);

int main(int argc, char **argv)
{
    if(argc < 14)return 1;
    signal(SIGINT, on_sigint);

    const long cam_from = atol(argv[1]), cam_to = atol(argv[2]);
    const long imu_from = atol(argv[3]), imu_to = atol(argv[4]);
    const char *dir = argv[5];
    const bool stitch = (atoi(argv[6]) != 0);
    const bool keep_cam = (cam_from >= 0), keep_imu = (imu_from >= 0);

    if(!keep_cam && !keep_imu){
        fprintf(stderr, "nothing to record\n");
        return 1;
    }
    if(keep_cam && cam_to - cam_from > MAX_FRAMES){
        fprintf(stderr, "camera range is %ld frames, the limit is %d\n", cam_to - cam_from, MAX_FRAMES);
        return 1;
    }

    const size_t cam_cap = keep_cam ? (size_t)(cam_to - cam_from) : 0;
    const size_t imu_cap = keep_imu ? (size_t)(imu_to - imu_from) : 0;
    gs130_image_nv12_t (*cam)[2] = cam_cap ? calloc(cam_cap, sizeof(*cam)) : NULL;
    gs130_imu_packet_t *imu = imu_cap ? calloc(imu_cap, sizeof(*imu)) : NULL;
    if((cam_cap && cam == NULL) || (imu_cap && imu == NULL)){
        fprintf(stderr, "out of memory\n");
        return 1;
    }
    size_t ncam = 0, nimu = 0;

    mkdir_p(dir);
    char cam_dir[512];
    snprintf(cam_dir, sizeof(cam_dir), "%s/cam", dir);
    if(keep_cam)mkdir_p(cam_dir);

    gs130_camera_mode_t mode = !strcmp(argv[9], "rect")   ? GS130_CAMERA_MODE_RECT :
                               !strcmp(argv[9], "resize") ? GS130_CAMERA_MODE_RESIZE :
                               GS130_CAMERA_MODE_RAW;

    gs130_config_t cfg = GS130_CONFIG(
        argv[7], argv[8], mode, atoi(argv[10]), atoi(argv[11]), atoi(argv[12]), atoi(argv[13]));
    if(stitch)cfg.camera_config.stereo_layout = GS130_STEREO_LAYOUT_LEFT_RIGHT;

    /* Create Device Handle */
    gs130_device_t *dev = gs130_create();

    /* Init Device and Start DataFlow */
    int ret = 0;
    if(gs130_init(dev, &cfg) != GS130_OK || gs130_start(dev) != GS130_OK){
        fprintf(stderr, "init or start failed\n");
        ret = 1; goto out;
    }

    printf("recording into %s: cam [%ld, %ld)%s, imu [%ld, %ld)\n", dir, cam_from, cam_to,
           stitch ? " stitched" : "", imu_from, imu_to);

    const uint64_t t0 = now_us();
    uint64_t cam_idx = 0, imu_idx = 0;

    /* Get data */
    while(!stop){
        gs130_image_nv12_t f[2];
        while(get_frame(dev, stitch, f)){
            const uint64_t i = cam_idx++;
            if(keep_cam && i >= (uint64_t)cam_from && i < (uint64_t)cam_to){
                cam[ncam][0] = f[0];
                cam[ncam][1] = f[1];
                ncam++;
            } else {
                free(f[0].data), free(f[1].data);   /* the buffer is ours to free */
            }
        }
        gs130_imu_packet_t p;
        while(gs130_get_imu_packet(dev, &p) == GS130_OK){
            const uint64_t i = imu_idx++;
            if(keep_imu && i >= (uint64_t)imu_from && i < (uint64_t)imu_to)imu[nimu++] = p;
        }

        report(t0, keep_cam, ncam, cam_cap, keep_imu, nimu, imu_cap);
        usleep(1000);
    }

    /* release the device before the write, so it is not held while the files go out */
    gs130_stop(dev);
    gs130_deinit(dev);
    gs130_destroy(dev);
    dev = NULL;   /* out: only tears the device down when init/start failed */
    printf("\nrecorded for %.3fs: kept %zu camera frames and %zu imu packets\n",
           (double)(now_us() - t0) / 1e6, ncam, nimu);

    /* the kept items are a contiguous run, so entry i is <range>_from + i */
    for(size_t i = 0; i < ncam; i++)
        for(int e = 0; e < (stitch ? 1 : 2); e++){
            save_frame(dir, cam_from + (long)i, stitch ? "" : (e ? "_R" : "_L"), &cam[i][e]);
            free(cam[i][e].data);
        }
    if(keep_cam)printf("wrote %s/cam: %zu frames\n", dir, ncam);
    if(keep_imu)save_imu(dir, imu_from, imu, nimu);

out:
    if(dev != NULL){
        gs130_deinit(dev);
        gs130_destroy(dev);
    }
    free(cam);
    free(imu);
    return ret;
}

/* pull one frame pair; when stitching there is one frame in f[0] and f[1] stays empty */
static bool get_frame(gs130_device_t *dev, bool stitch, gs130_image_nv12_t f[2])
{
    f[1] = (gs130_image_nv12_t){0};
    return stitch ? gs130_get_stereo_nv12_frame(dev, &f[0]) == GS130_OK
                  : gs130_get_nv12_frame(dev, &f[0], &f[1]) == GS130_OK;
}

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}

static void mkdir_p(const char *path)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);

    for(char *p = tmp + 1; *p != '\0'; p++){
        if(*p != '/')continue;
        *p = '\0';
        mkdir(tmp, 0777);
        *p = '/';
    }
    mkdir(tmp, 0777);
}

static void report(uint64_t t0, bool keep_cam, size_t ncam, size_t cam_cap,
                   bool keep_imu, size_t nimu, size_t imu_cap)
{
    static uint64_t next_report = 0;
    const uint64_t now = now_us();
    if(now < next_report)return;
    next_report += REPORT_US;
    if(next_report <= now)next_report = now + REPORT_US;

    printf("[%6.1fs] ", (double)(now - t0) / 1e6);
    if(keep_cam)printf("cam %zu/%zu   ", ncam, cam_cap);
    if(keep_imu)printf("imu %zu/%zu", nimu, imu_cap);
    printf("\n");
}

static void save_frame(const char *dir, long index, const char *suffix, const gs130_image_nv12_t *image)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/cam/%06ld%s.nv12", dir, index, suffix);

    if(image->data == NULL)return;

    FILE *f = fopen(path, "wb");
    if(f == NULL){
        fprintf(stderr, "cannot open %s\n", path);
        return;
    }
    fwrite(image->data, 1, (size_t)image->width * image->height * 3 / 2, f);
    fclose(f);
}

static void save_imu(const char *dir, long from, const gs130_imu_packet_t *imu, size_t n)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/imu.csv", dir);

    FILE *f = fopen(path, "w");
    if(f == NULL){
        fprintf(stderr, "cannot open %s\n", path);
        return;
    }

    fprintf(f, "index,timestamp_ns,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,temp,is_fsync\n");
    for(size_t i = 0; i < n; i++)
        fprintf(f, "%ld,%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%d\n", from + (long)i,
                (unsigned long long)imu[i].timestamp_ns, imu[i].accel[0], imu[i].accel[1],
                imu[i].accel[2], imu[i].gyro[0], imu[i].gyro[1], imu[i].gyro[2],
                imu[i].temp, imu[i].is_fsync);
    fclose(f);

    printf("wrote %s: %zu packets\n", path, n);
}
