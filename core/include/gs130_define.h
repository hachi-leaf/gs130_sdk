/**
 * @file gs130_define.h
 * @brief Optional customer-facing preset configuration macros.
 *
 * NOTE: the preset values in this file may change between SDK versions;
 *       no backward compatibility is guaranteed for this file.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEFINE_H
#define GS130_DEFINE_H

/* Preset dispatcher: GS130_CONFIG(RDKX5, GS130WI, mode, w, h, fps, odr)
 * forwards to GS130_CONFIG_<platform>_<model>(mode, w, h, fps, odr).
 * An undefined platform/model combination fails the build, which is intended. */
#define GS130_CONFIG(platform, model, ...)  GS130_CONFIG_##platform##_##model(__VA_ARGS__)

/* RDK X5 + GS130WI preset: board-fixed fields use the verified bring-up values
 * (sensor 1088x1280, buses 4/6, mipi/gpio maps, EEPROM 0x50, fifos);
 * the commonly-tuned knobs are parameters.
 *
 *   gs130_config_t cfg = GS130_CONFIG_RDKX5_GS130WI(GS130_CAMERA_MODE_RECT, 544, 640, 30, 200);
 *   gs130_init(dev, &cfg);
 *
 * NOTE: uses GNU range designated initializers ([a ... b] = v); fine on gcc/clang.
 * NOTE: fps_ only sets the frame rate request; line_length/frame_length stay at the
 *       bring-up values, so large fps changes may need those adjusted too. */
#define GS130_CONFIG_RDKX5_GS130WI(mode_, width_, height_, fps_, odr_) {    \
    .camera_config = {                                                      \
        .bus = {4, 6}, .bus_num = 2,                                        \
        .left_addr = 0x30, .right_addr = 0x32,                              \
        .sensor_width = 1088, .sensor_height = 1280,                        \
        .fps = (fps_), .line_length = 1400, .frame_length = 1500,           \
        .tuning_file = NULL,                                                \
        .output_width = (width_), .output_height = (height_),               \
        .mode = (mode_),                                                    \
        .stereo_layout = GS130_STEREO_LAYOUT_NONE,                          \
        .bus_mipi_rx    = { [0 ... 31] = 0xFF, [4] = 2, [6] = 0 },         \
        .bus_reset_gpio = { [0 ... 31] = -1,   [4] = 351, [6] = 353 },     \
        .fsync_camera = GS130_CAMERA_RIGHT_IDX,                             \
    },                                                                      \
    .imu_config = {                                                         \
        .bus = {4, 6}, .bus_num = 2, .addr = 0x68,                          \
        .odr_hz = (odr_), .accel_fsr_g = 16, .gyro_fsr_dps = 2000,          \
        .accel_bw_sel = 0, .gyro_bw_sel = 0,                                \
    },                                                                      \
    .eeprom_config = { .bus = {4, 6}, .bus_num = 2, .addr = 0x50 },         \
    .camera_fifo = { .depth = 4,    .mode = GS130_FIFO_DROP_OLD },          \
    .imu_fifo    = { .depth = 1024, .mode = GS130_FIFO_DROP_OLD },          \
}

#endif /* GS130_DEFINE_H */
