/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef GS130_H
#define GS130_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum gs130_err_e{
    GS130_OK = 0,
    GS130_PARAM_ERROR,
    GS130_UNSUPPORTED,
    GS130_NOT_FOUND,
    GS130_HW_ERROR,
    GS130_TIMEOUT,
}gs130_err_t;

/* FIFO 队列模式 */
typedef enum gs130_fifo_mode_e{
    GS130_FIFO_DROP_NEW,   /* 满则丢弃新数据 */
    GS130_FIFO_DROP_OLD,   /* 满则覆盖最旧数据 */
}gs130_fifo_mode_t;

/* FIFO 队列配置 */
typedef struct gs130_fifo_config_s{
    size_t depth;              /* 队列深度，≥2 */
    gs130_fifo_mode_t mode;   /* 满时策略 */
}gs130_fifo_config_t;

/* ==================== Device Config ==================== */

typedef enum gs130_camera_mode_e{
    GS130_CAMERA_MODE_RAW,   
    GS130_CAMERA_MODE_RESIZE,
    GS130_CAMERA_MODE_RECT,  
}gs130_camera_mode_t;

typedef enum {
    GS130_CAMERA_RIGHT_IDX = 0,
    GS130_CAMERA_LEFT_IDX  = 1,
} gs130_camera_index_t;

typedef struct {
    uint8_t bus[32];
    size_t bus_num;
    uint8_t  left_addr;
    uint8_t  right_addr;

    uint32_t sensor_width, sensor_height;
    uint32_t fps;
    uint32_t line_length, frame_length;
    const char *tuning_file;   /* ISP tuning，nullptr = 不加载 */
    
    uint32_t output_width, output_height;
    gs130_camera_mode_t mode;

    uint8_t bus_mipi_rx[32];
    int bus_reset_gpio[32];

    gs130_camera_index_t fsync_camera;   /* IMU FSYNC 脚绑定的目 */
}gs130_camera_config_t;

typedef struct gs130_imu_config_s{
    uint8_t bus[32];
    size_t bus_num;
    uint8_t addr;
    uint32_t odr_hz;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint8_t accel_bw_sel;   /* UI 滤波档位 0..7，对应带宽见 info() */
    uint8_t gyro_bw_sel;    /* 0xFF = 未设置 */
}gs130_imu_config_t;

typedef struct gs130_eeprom_config_s{
    uint8_t bus[32];
    size_t bus_num;
    uint8_t addr;
}gs130_eeprom_config_t;

typedef struct gs130_config_s{
    gs130_camera_config_t camera_config;
    gs130_imu_config_t imu_config;
    gs130_eeprom_config_t eeprom_config;

    /* FIFO 队列：相机帧 / IMU 包分开设置 */
    gs130_fifo_config_t camera_fifo;
    gs130_fifo_config_t imu_fifo;
}gs130_config_t;

typedef struct gs130_device_s gs130_device_t;

gs130_device_t *gs130_create();

void gs130_destroy(
    gs130_device_t *dev);

gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg);

gs130_err_t gs130_deinit(
    gs130_device_t *dev);

gs130_err_t gs130_start(
    gs130_device_t *dev);

void gs130_stop(
    gs130_device_t *dev);

/* ==================== Camera Data ==================== */

typedef struct gs130_image_nv12_s{
    uint8_t *y, *uv;
    uint32_t width, height;
    uint32_t y_stride, uv_stride;
    uint64_t timestamp_ns;
}gs130_image_nv12_t;

size_t gs130_available_camera(
    gs130_device_t *dev);

gs130_err_t gs130_get_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image_left,
    gs130_image_nv12_t *image_right);

/* ==================== IMU Data ==================== */

typedef struct gs130_imu_packet_s{
    float accel[3]; /* m/s² */
    float gyro[3]; /* rad/s */
    float temp; /* °C */
    bool  is_fsync; /* 本包为 FSYNC 同步包 */
    uint64_t timestamp_ns; /* 修正后的绝对时间戳（与相机时钟对齐） */
}gs130_imu_packet_t;

size_t gs130_available_imu(
    gs130_device_t *dev);

gs130_err_t gs130_read_imu(
    gs130_device_t *dev,
    gs130_imu_packet_t *out);

/* ==================== EEprom Data ==================== */

typedef enum gs130_dist_model_e{
    GS130_DIST_PINHOLE,    /* dist_coeffs = [k1,k2,p1,p2,k3,k4,k5,k6] */
    GS130_DIST_FISHEYE,    /* dist_coeffs = [k1,k2,k3,k4,0,0,0,0] */
}gs130_dist_model_t;

typedef struct gs130_camera_intrinsics_s{
    double fx, fy, cx, cy;
    double K[9], dist_coeffs[8];
    gs130_dist_model_t dist_model;
}gs130_camera_intrinsics_t;

typedef struct gs130_imu_intrinsics_s{
    double accel_misalign[9];
    double accel_scale[3];
    double accel_bias[3];
    double accel_noise;          /* m/s²/√Hz */
    double accel_random_walk;    /* m/s³/√Hz */
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           /* rad/s/√Hz */
    double gyro_random_walk;     /* rad/s²/√Hz */
}gs130_imu_intrinsics_t;

/* 外参为该传感器坐标系 → 公共参考坐标系的变换：p_ref = R*p_sensor + T */
typedef struct {
    gs130_imu_intrinsics_t imu;
    gs130_camera_intrinsics_t camera_right;
    gs130_camera_intrinsics_t camera_left;

    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];
    int camera_install_angle;
} gs130_calibration_t;

gs130_err_t gs130_get_calibration(
    gs130_device_t *dev,
    gs130_calibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif /* GS130_H */
