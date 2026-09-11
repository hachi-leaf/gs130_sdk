/**
 * @file gs130-calib-export.c
 * @brief Export script
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include "gs130_define.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

static void save_yaml(const char *dir, const char *name, char *yaml);
static char *camchain_yaml(gs130_device_t *dev, const gs130_calibration_t *cal,
                           int w, int h, bool has_imu);
static char *imu_yaml(const gs130_calibration_t *cal, int odr);

int main(int argc, char **argv)
{
    (void)argc;

    gs130_camera_mode_t mode = !strcmp(argv[4], "rect")   ? GS130_CAMERA_MODE_RECT :
                               !strcmp(argv[4], "resize") ? GS130_CAMERA_MODE_RESIZE :
                               GS130_CAMERA_MODE_RAW;

    gs130_config_t cfg = GS130_CONFIG(
        argv[2], argv[3], mode, atoi(argv[5]), atoi(argv[6]), atoi(argv[7]), atoi(argv[8]));

    /* Create Device Handle */
    gs130_device_t *dev = gs130_create();

    /* Init Device */
    int ret = 0;
    if(gs130_init(dev, &cfg) != GS130_OK){
        fprintf(stderr, "init failed\n");
        ret = 1; goto out;
    }

    gs130_calibration_t cal;
    if(gs130_get_calibration(dev, &cal) != GS130_OK){
        fprintf(stderr, "get_calibration failed\n");
        ret = 1; goto out;
    }

    /* Get stereo (and IMU) calibration datas */
    if(strcmp(argv[4], "raw") != 0){
        fprintf(stderr, "\033[33mwarning: mode is \"%s\", not \"raw\" -- the exported calibration is the "
            "rectified/resized one, not the EEPROM's raw calibration\033[0m\n", argv[4]);
    }
    char *camchain = camchain_yaml(
        dev, &cal, 
        cfg.camera_config.output_width,
        cfg.camera_config.output_height,
        gs130_get_imu_name(dev) != NULL);
    printf("%s", camchain);
    save_yaml(argv[1], "camchain.yaml", camchain);

    /* Get IMU intrinsics and save */
    if(gs130_get_imu_name(dev) != NULL){
        char *imu = imu_yaml(&cal, atoi(argv[8]));
        printf("%s", imu);
        save_yaml(argv[1], "imu.yaml", imu);
    }


out:
    gs130_deinit(dev);
    gs130_destroy(dev);
    return ret;
}

static char *camchain_yaml(gs130_device_t *dev, const gs130_calibration_t *cal,
                           int w, int h, bool has_imu)
{
    char *yaml = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&yaml, &len);
    if(f == NULL)return NULL;

    fprintf(f, "# gs130 EEPROM calibration, Kalibr camchain format\n");
    fprintf(f, "# p_to = T * p_from (T_cam_imu: IMU -> camera, T_cn_cnm1: cam0 -> cam1)\n");

    for(int i = 0; i < 2; i++){
        const gs130_camera_intrinsics_t *ci = i ? &cal->camera_left : &cal->camera_right;
        const gs130_reference_frame_t ref = i ? GS130_REF_CAMERA_LEFT : GS130_REF_CAMERA_RIGHT;
        double R[9], T[3];

        fprintf(f, "cam%d:\n", i);
        fprintf(f, "  camera_model: pinhole\n");
        fprintf(f, "  distortion_model: %s\n",
                (ci->dist_model == GS130_DIST_FISHEYE) ? "equidistant" : "radtan");
        /* Kalibr takes 4 coefficients: radtan=[k1,k2,p1,p2], equidistant=[k1,k2,k3,k4] */
        fprintf(f, "  distortion_coeffs: [%.12g, %.12g, %.12g, %.12g]\n",
                ci->dist_coeffs[0], ci->dist_coeffs[1], ci->dist_coeffs[2], ci->dist_coeffs[3]);
        fprintf(f, "  intrinsics: [%.12g, %.12g, %.12g, %.12g]\n", ci->fx, ci->fy, ci->cx, ci->cy);
        fprintf(f, "  resolution: [%d, %d]\n", w, h);
        fprintf(f, "  rostopic: /cam%d/image_raw\n", i);

        if(has_imu){
            gs130_get_relative_R(dev, GS130_REF_IMU, ref, R);
            gs130_get_relative_T(dev, GS130_REF_IMU, ref, T);
            fprintf(f, "  T_cam_imu:\n");
            for(int r = 0; r < 3; r++)
                fprintf(f, "  - [%.12g, %.12g, %.12g, %.12g]\n",
                        R[r * 3], R[r * 3 + 1], R[r * 3 + 2], T[r]);
            fprintf(f, "  - [0.0, 0.0, 0.0, 1.0]\n");
        }
        if(i){
            /* cam1 (left) from cam0 (right) */
            gs130_get_relative_R(dev, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, R);
            gs130_get_relative_T(dev, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, T);
            fprintf(f, "  T_cn_cnm1:\n");
            for(int r = 0; r < 3; r++)
                fprintf(f, "  - [%.12g, %.12g, %.12g, %.12g]\n",
                        R[r * 3], R[r * 3 + 1], R[r * 3 + 2], T[r]);
            fprintf(f, "  - [0.0, 0.0, 0.0, 1.0]\n");
        }
    }

    fclose(f);
    return yaml;
}

static char *imu_yaml(const gs130_calibration_t *cal, int odr)
{
    char *yaml = NULL;
    size_t len = 0;
    FILE *f = open_memstream(&yaml, &len);
    if(f == NULL)return NULL;

    const gs130_imu_intrinsics_t *imu = &cal->imu;

    fprintf(f, "# gs130 EEPROM calibration, IMU parameters\n");
    fprintf(f, "accelerometer_noise_density: %.12g\n", imu->accel_noise);
    fprintf(f, "accelerometer_random_walk:   %.12g\n", imu->accel_random_walk);
    fprintf(f, "gyroscope_noise_density:     %.12g\n", imu->gyro_noise);
    fprintf(f, "gyroscope_random_walk:       %.12g\n", imu->gyro_random_walk);
    fprintf(f, "rostopic: /imu0\n");
    fprintf(f, "update_rate: %d.0\n", odr);

    fprintf(f, "\n# --- gs130 extension: IMU intrinsics (not part of Kalibr's schema) ---\n");

    for(int i = 0; i < 2; i++){
        const double *mis   = i ? imu->gyro_misalign : imu->accel_misalign;
        const double *scale = i ? imu->gyro_scale    : imu->accel_scale;
        const double *bias  = i ? imu->gyro_bias     : imu->accel_bias;

        fprintf(f, "%s:\n", i ? "gyroscope" : "accelerometer");
        fprintf(f, "  misalignment: [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g]\n",
                mis[0], mis[1], mis[2], mis[3], mis[4], mis[5], mis[6], mis[7], mis[8]);
        fprintf(f, "  scale: [%.12g, %.12g, %.12g]\n", scale[0], scale[1], scale[2]);
        fprintf(f, "  bias: [%.12g, %.12g, %.12g]\n", bias[0], bias[1], bias[2]);
    }

    fclose(f);
    return yaml;
}

static void save_yaml(const char *dir, const char *name, char *yaml)
{
    char path[512];
    snprintf(path, sizeof(path), "%s/%s", dir, name);

    for(char *p = path + 1; *p != '\0'; p++){
        if(*p != '/')continue;
        *p = '\0';
        mkdir(path, 0777);
        *p = '/';
    }

    FILE *f = fopen(path, "w");
    if(f != NULL){
        fputs(yaml, f);
        fclose(f);
    }
    free(yaml);
}
