/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * SDK-wide type definitions, all collected in this file.
 * Data types only, no behavior: each module's class and ModelDesc stay in its own header.
 */
#ifndef GS130_TYPES_HPP
#define GS130_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gs130 {

// ============================== Error codes ==============================

enum class Status {
    Ok,
    ParamError,     // the parameter itself is invalid (null pointer, required field is 0, etc.)
    Unsupported,    // hardware cannot do this configuration; never degrade, round, or substitute
    NotFound,       // device or model not detected
    HwError,        // low-level communication or driver failure
    Timeout,        // wait timed out
};

// ============================== Camera & output ==============================

enum class CamIndex {
    Right = 0,
    Left  = 1,
    Num   = 2,
};

enum class OutputMode {
    Raw,      // CAM -> VIN -> ISP -> OUT
    Resize,   // CAM -> VIN -> ISP -> VSE -> OUT
    Rect,     // CAM -> VIN -> ISP -> GDC -> VSE -> OUT
};

struct PipelineConfig {
    // Platform mapping: I2C bus -> MIPI RX / reset GPIO.
    // Looked up by the bus the camera is detected on; buses may be swapped, so key by bus, not camera index.
    // bus_mipi_rx 0xFF = not configured; bus_reset_gpio -1 = do not control.
    uint8_t bus_mipi_rx[32];
    int     bus_reset_gpio[32];

    // sensor output size
    uint32_t sensor_width, sensor_height;

    uint32_t fps;
    OutputMode mode;

    // Required for Resize / Rect modes; in Raw mode must equal the sensor size.
    uint32_t output_width;
    uint32_t output_height;

    uint32_t line_length;
    uint32_t frame_length;

    const char *tuning_file;   // ISP tuning, nullptr = do not load
};

// ============================== IMU ==============================

// ICM-family IMU 16-byte FIFO packet layout
struct ImuHwFifo16Packet {
    int16_t  accel[3];
    int16_t  gyro[3];
    int8_t   temp;
    bool     is_fsync;
    uint32_t delta_time_us;   // valid only when is_fsync: edge-to-sample offset
};

struct ImuConfig {
    uint32_t odr_hz;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint8_t  accel_bw_sel;   // UI filter level 0..7; see info() for the bandwidths
    uint8_t  gyro_bw_sel;    // 0xFF = not set
};

// ============================== Calibration ==============================

// Distortion model
enum class DistModel {
    Pinhole,        // dist_coeffs = [k1,k2,p1,p2,k3,k4,k5,k6]
    Fisheye,        // dist_coeffs = [k1,k2,k3,k4,0,0,0,0]
};

struct CameraIntrinsics {
    double fx, fy, cx, cy;
    double K[9], dist_coeffs[8];
    DistModel dist_model = DistModel::Fisheye;
};

struct ImuIntrinsics {
    double accel_misalign[9];    // cross-axis coupling 3x3
    double accel_scale[3];
    double accel_bias[3];
    double accel_noise;          // m/s²/√Hz
    double accel_random_walk;    // m/s³/√Hz
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           // rad/s/√Hz
    double gyro_random_walk;     // rad/s²/√Hz
};

// Each of the three extrinsic sets is a "sensor frame -> common reference frame"
// transform, i.e. p_ref = R * p_sensor + T, not a transform between devices
struct StereoImuModel {
    ImuIntrinsics    imu;
    CameraIntrinsics cam_right;
    CameraIntrinsics cam_left;

    double cam_right_R[9];
    double cam_right_T[3];
    double cam_left_R[9];
    double cam_left_T[3];
    double imu_R[9];
    double imu_T[3];
    int    install_angle;
};

// ============================== Stereo rectification ==============================

// Remap coordinate point: output pixel -> source image sampling coordinate.
// Layout matches point_t{double x,y} in Horizon gdc_bin_cfg.h;
// the platform layer static_asserts it, then passes it to GDC zero-copy.
struct RemapPoint {
    double x, y;
};

// ============================== FIFO ==============================

enum class FifoMode {
    DropNew,   // drop new data when full
    DropOld,   // overwrite oldest data when full
};

} // namespace gs130

#endif // GS130_TYPES_HPP
