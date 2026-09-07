/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * 全 SDK 类型定义，统一放在此文件。
 * 只放数据类型，不放行为：各模块的 class 与 ModelDesc 留在自己的头文件里。
 */
#ifndef GS130W_TYPES_HPP
#define GS130W_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace gs130w {

// ============================== 错误码 ==============================

enum class Status {
    Ok,
    ParamError,     // 参数本身非法（空指针、必填项为 0 等）
    Unsupported,    // 硬件做不到该配置；绝不降级、不取整、不替换
    NotFound,       // 未探测到器件或型号
    HwError,        // 底层通信或驱动失败
    Timeout,        // 等待超时
};

// ============================== 相机与出图 ==============================

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
    // 平台映射：I2C 总线 → MIPI RX / 复位 GPIO。
    // 探测出相机落在哪个总线后据此取值；总线可能互换，故按总线而非相机索引。
    // bus_mipi_rx 0xFF = 未配置；bus_reset_gpio -1 = 不控制。
    uint8_t bus_mipi_rx[32];
    int     bus_reset_gpio[32];

    // sensor 出图尺寸
    uint32_t sensor_width, sensor_height;

    uint32_t fps;
    OutputMode mode;

    // Resize / Rect 模式必须给出；Raw 模式必须等于 sensor 尺寸。
    uint32_t output_width;
    uint32_t output_height;

    uint32_t line_length;
    uint32_t frame_length;

    const char *tuning_file;   // ISP tuning，nullptr = 不加载
};

// ============================== IMU ==============================

// ICM 族 IMU 16 字节 FIFO 包结构
struct ImuHwFifo16Packet {
    int16_t  accel[3];
    int16_t  gyro[3];
    int8_t   temp;
    bool     is_fsync;
    uint32_t delta_time_us;   // 仅 is_fsync 时有效：沿到采样点偏移
};

struct ImuConfig {
    uint32_t odr_hz;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint8_t  accel_bw_sel;   // UI 滤波档位 0..7，对应带宽见 info()
    uint8_t  gyro_bw_sel;    // 0xFF = 未设置
};

// ============================== 标定 ==============================

// 畸变模型
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
    double accel_misalign[9];    // 交叉轴耦合 3×3
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

// 三组外参各为「该传感器坐标系 → 公共参考坐标系」的变换
// 即 p_ref = R * p_sensor + T，并非设备之间的变换
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

// ============================== 立体校正 ==============================

// 重映射坐标点：输出像素 → 源图采样坐标。
// 布局与地平线 gdc_bin_cfg.h 的 point_t{double x,y} 一致，
// 平台层经 static_assert 校验后可零拷贝传给 GDC。
struct RemapPoint {
    double x, y;
};

// ============================== FIFO ==============================

enum class FifoMode {
    DropNew,   // 满则丢弃新数据
    DropOld,   // 满则覆盖最旧数据
};

} // namespace gs130w

#endif // GS130W_TYPES_HPP
