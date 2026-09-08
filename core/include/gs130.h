/**
 * @file gs130.h
 * @brief GS130xx SDK C API.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
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
    GS130_OK = 0,          /* success */
    GS130_PARAM_ERROR,     /* invalid parameter (null pointer, bad config, wrong call order) */
    GS130_UNSUPPORTED,     /* hardware cannot do this configuration */
    GS130_NOT_FOUND,       /* device or calibration data not detected */
    GS130_HW_ERROR,        /* low-level communication or driver failure */
    GS130_TIMEOUT,         /* no data available right now (non-blocking reads) */
    GS130_THREAD_CLOSED,   /* threads closed or never started; after a fault, recover via deinit + init */
}gs130_err_t;

typedef enum gs130_fifo_mode_e{
    GS130_FIFO_DROP_NEW,   /* drop the newest data when full */
    GS130_FIFO_DROP_OLD,   /* overwrite the oldest data when full */
}gs130_fifo_mode_t;

typedef struct gs130_fifo_config_s{
    size_t depth;              /* queue depth, >= 2 */
    gs130_fifo_mode_t mode;   /* policy when full */
}gs130_fifo_config_t;

/* ==================== Device Config ==================== */

typedef enum gs130_camera_mode_e{
    GS130_CAMERA_MODE_RAW,     /* CAM -> VIN -> ISP -> OUT */
    GS130_CAMERA_MODE_RESIZE,  /* CAM -> VIN -> ISP -> VSE -> OUT */
    GS130_CAMERA_MODE_RECT,    /* CAM -> VIN -> ISP -> GDC -> VSE -> OUT (needs calibration) */
}gs130_camera_mode_t;

typedef enum gs130_camera_index_e{
    GS130_CAMERA_RIGHT_IDX = 0,
    GS130_CAMERA_LEFT_IDX  = 1,
}gs130_camera_index_t;

/** 双目拼接方式。 */
typedef enum gs130_stereo_layout_e{
    GS130_STEREO_LAYOUT_NONE,          /* 不拼接（默认）：双目分离帧输出 */
    GS130_STEREO_LAYOUT_LEFT_RIGHT,    /* 横向拼接：左目在左，右目在右 */
    GS130_STEREO_LAYOUT_RIGHT_LEFT,    /* 横向拼接：右目在左，左目在右 */
    GS130_STEREO_LAYOUT_TOP_BOTTOM,    /* 纵向拼接：左目在上，右目在下 */
    GS130_STEREO_LAYOUT_BOTTOM_TOP,    /* 纵向拼接：右目在上，左目在下 */
}gs130_stereo_layout_t;

typedef struct gs130_camera_config_s{
    uint8_t bus[32]; /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t  left_addr;
    uint8_t  right_addr;

    uint32_t sensor_width, sensor_height;
    uint32_t fps;
    uint32_t line_length, frame_length;
    const char *tuning_file; /* ISP tuning file, nullptr = do not load */

    uint32_t output_width, output_height;   /* Raw mode: must equal the sensor size */
    gs130_camera_mode_t mode;

    /* 拼接输出：GS130_STEREO_LAYOUT_NONE（默认）= 双目分离帧；
       其余值 = 相机线程直接按布局拼接填充，拼接取帧零拷贝 */
    gs130_stereo_layout_t stereo_layout;

    uint8_t bus_mipi_rx[32]; /* I2C bus -> MIPI RX map; 0xFF = not configured */
    int bus_reset_gpio[32]; /* I2C bus -> reset GPIO map; -1 = do not control */
    /* e.g. bus-4 & gpio-532 & mipi-rx-1 */
    /* memset(bus_mipi_rx, 0xFF, sizeof(bus_mipi_rx)); */
    /* memset(bus_reset_gpio, -1, sizeof(bus_reset_gpio)); */
    /* bus_mipi_rx[4] = 1; */
    /* bus_reset_gpio[4] = 532; */

    gs130_camera_index_t fsync_camera;   /* camera the IMU FSYNC pin is bound to */
}gs130_camera_config_t;

typedef struct gs130_imu_config_s{
    uint8_t bus[32];        /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t addr;
    uint32_t odr_hz;
    uint16_t accel_fsr_g;
    uint16_t gyro_fsr_dps;
    uint8_t accel_bw_sel;   /* UI filter level 0..7, see info() for bandwidth */
    uint8_t gyro_bw_sel;    /* 0xFF = not set */
}gs130_imu_config_t;

/** EEPROM (calibration data) configuration. */
typedef struct gs130_eeprom_config_s{
    uint8_t bus[32];        /* candidate I2C buses, probed in order */
    size_t bus_num;
    uint8_t addr;
}gs130_eeprom_config_t;

/** Top-level device configuration for gs130_init(). */
typedef struct gs130_config_s{
    gs130_camera_config_t camera_config;
    gs130_imu_config_t imu_config;
    gs130_eeprom_config_t eeprom_config;

    /* FIFO queues: camera frames and IMU packets configured separately */
    gs130_fifo_config_t camera_fifo;
    gs130_fifo_config_t imu_fifo;
}gs130_config_t;

/** Opaque device handle. */
typedef struct gs130_device_s gs130_device_t;

/**
 * @brief 创建设备句柄（空壳，需再调用 gs130_init 完成初始化）。
 *
 * @return 设备句柄；使用完毕后由 gs130_destroy() 释放。
 */
gs130_device_t *gs130_create();

/**
 * @brief 释放设备句柄。
 *
 * 调用前必须先调用 gs130_deinit()（或至少 gs130_stop()），
 * 在后台线程仍在运行时销毁句柄属于未定义行为。
 *
 * @param[in] dev Device handle from gs130_create().
 */
void gs130_destroy(
    gs130_device_t *dev);

/**
 * @brief 初始化设备：按候选总线顺序探测 EEPROM/IMU、加载标定、配置相机与 IMU（不开流）。
 *
 * EEPROM 与 IMU 均为可选，未探测到时 SDK 降级运行；
 * 但 GS130_CAMERA_MODE_RECT 模式必须有标定，无 EEPROM 时返回 GS130_PARAM_ERROR。
 * 每个句柄只能初始化一次，重复调用返回 GS130_PARAM_ERROR。
 *
 * @param[in] dev Device handle from gs130_create().
 * @param[in] cfg 设备配置，见 gs130_config_t。
 * @return err code
 */
gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg);

/**
 * @brief 反初始化：释放所有设备资源（仍在运行时会先调用 gs130_stop()）。
 *
 * 调用后句柄本身仍然有效，需由 gs130_destroy() 释放。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return err code
 */
gs130_err_t gs130_deinit(
    gs130_device_t *dev);

/**
 * @brief 开始采集。
 *
 * 检测到 IMU 时，相机等待 IMU FSYNC 握手完成后再开流；未检测到 IMU 时直接开流。
 * 重复调用返回 GS130_PARAM_ERROR。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return err code
 */
gs130_err_t gs130_start(
    gs130_device_t *dev);

/**
 * @brief 停止采集并等待后台线程退出。
 *
 * 未在运行时调用为空操作；停止后可再次调用 gs130_start()。
 *
 * @param[in] dev Device handle from gs130_create().
 */
void gs130_stop(
    gs130_device_t *dev);

/* ==================== Camera Data ==================== */

typedef struct gs130_image_nv12_s{
    uint8_t *data; /* contiguous tightly-packed NV12: Y plane (width*height bytes), then UV plane (width*height/2 bytes) */
    uint32_t width, height;
    uint64_t timestamp_ns;
}gs130_image_nv12_t;

/**
 * @brief 查询当前队列中可获取的双目同步帧对数量。
 *
 * 与 gs130_get_nv12_frame 和 gs130_get_stereo_nv12_frame 配合使用：数量大于 0 时才调用获取。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return 队列中可取的帧对数量。
 */
size_t gs130_available_camera(
    gs130_device_t *dev);

/**
 * @brief 分别获取左右目 NV12 同步帧。
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] image_left Left camera frame.
 * @param[out] image_right Right camera frame.
 * @return err code
 *
 * @note image_left / image_right 的 data 缓冲区由 SDK 使用 malloc() 分配，所有权归调用方，使用后需自行 free()。
 * @note 拼接模式（stereo_layout 不为 NONE）下不可用，返回 GS130_UNSUPPORTED。
 */
gs130_err_t gs130_get_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image_left,
    gs130_image_nv12_t *image_right);

/**
 * @brief 获取左右目拼接的 NV12 同步帧。
 *
 * 拼接布局由 gs130_camera_config_t.stereo_layout 在 gs130_init 时定死；
 * 相机线程直接按布局填充，本函数零拷贝弹出。
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] image 拼接后的单帧 NV12：横向拼接宽度翻倍，纵向拼接高度翻倍。
 * @return err code
 *
 * @note image 的 data 缓冲区由 SDK 使用 malloc() 分配，所有权归调用方，使用后需自行 free()。
 * @note 仅当 stereo_layout 不为 GS130_STEREO_LAYOUT_NONE 时可用，否则返回 GS130_UNSUPPORTED。
 */
gs130_err_t gs130_get_stereo_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image);

/* ==================== IMU Data ==================== */

typedef struct gs130_imu_packet_s{
    float accel[3]; /* m/s^2 */
    float gyro[3]; /* rad/s */
    float temp; /* degC */
    bool  is_fsync; /* this packet is an FSYNC sync packet */
    uint64_t timestamp_ns; /* corrected absolute timestamp (aligned to the camera clock) */
}gs130_imu_packet_t;

/**
 * @brief 查询当前队列中可获取的 IMU 数据包数量。
 *
 * 与 gs130_get_imu_packet 配合使用：数量大于 0 时才调用获取。
 * 未检测到 IMU 或 IMU 故障时返回 0。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return 队列中可取的数据包数量。
 */
size_t gs130_available_imu(
    gs130_device_t *dev);

/**
 * @brief 获取一个 IMU 数据包
 *
 * @param[in]  dev Device handle from gs130_create().
 * @param[out] out IMU packet.
 * @return err code
 */
gs130_err_t gs130_get_imu_packet(
    gs130_device_t *dev,
    gs130_imu_packet_t *out);

/**
 * @brief 获取探测到的 IMU 型号名。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return IMU 型号，字符串，未检测到 IMU 时返回 NULL。
 */
const char *gs130_get_imu_name(
    gs130_device_t *dev);

/**
 * @brief 获取探测到的 IMU 详细信息（档位带宽对照等）。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return IMU 信息，字符串，未检测到 IMU 时返回 NULL。
 */
const char *gs130_get_imu_info(
    gs130_device_t *dev);

/* ==================== EEPROM Data ==================== */

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
    double accel_noise;          /* m/s^2/sqrt(Hz) */
    double accel_random_walk;    /* m/s^3/sqrt(Hz) */
    double gyro_misalign[9];
    double gyro_scale[3];
    double gyro_bias[3];
    double gyro_noise;           /* rad/s/sqrt(Hz) */
    double gyro_random_walk;     /* rad/s^2/sqrt(Hz) */
}gs130_imu_intrinsics_t;

typedef struct gs130_calibration_s{
    gs130_imu_intrinsics_t imu;
    gs130_camera_intrinsics_t camera_right;
    gs130_camera_intrinsics_t camera_left;

    /** 外参说明
     * 1. 外参的含义为：将该设备坐标系下的点转换到参考坐标系下的旋转矩阵和位移向量，也就是参考坐标系下的绝对位姿
     * 2. 设备初始化时会定义参考坐标系（由 eeprom 驱动程序定义），并将所有设备的外参转换为该参考坐标系下的绝对位姿
     * 3. 开启双目立体矫正时，参考系不会变化，但外参会变成虚拟的（双目平行），所以在使用外参时要注意参考系的变化
     */
    double camera_right_R[9], camera_right_T[3];
    double camera_left_R[9], camera_left_T[3];
    double imu_R[9], imu_T[3];

    int camera_install_angle;
}gs130_calibration_t;

typedef enum gs130_reference_frame_e{
    GS130_REF_CAMERA_RIGHT,
    GS130_REF_CAMERA_LEFT,
    GS130_REF_IMU,
}gs130_reference_frame_t;

/**
 * @brief 获取相机内参。
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  cam_idx    Camera index.
 * @param[out] intrinsics Camera intrinsics.
 * @return err code
 */
gs130_err_t gs130_get_camera_intrinsics(
    gs130_device_t *dev,
    gs130_camera_index_t cam_idx,
    gs130_camera_intrinsics_t *intrinsics);

/**
 * @brief 获取 IMU 内参。
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[out] intrinsics IMU intrinsics.
 * @return err code
 */
gs130_err_t gs130_get_imu_intrinsics(
    gs130_device_t *dev,
    gs130_imu_intrinsics_t *intrinsics);

/**
 * @brief 获取指定设备之间的相对旋转矩阵。
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  from_frame Source frame.
 * @param[in]  to_frame   Target frame.
 * @param[out] R          Rotation matrix, row-major 3x3 (9 doubles).
 * @return err code
 *
 * @note 当 from_frame == to_frame 时，理论上会返回单位矩阵，但函数内部还是会进行矩阵运算，所以返回的单位阵可能有精度差异。
 * @note 输出 R 矩阵的含义为：将 from_frame 坐标系下的点转换到 to_frame 坐标系下的旋转矩阵，也就是以 to_frame 为参考系时 from_frame 的旋转。
 */
gs130_err_t gs130_get_relative_R(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *R);

/**
 * @brief 获取指定设备之间的相对平移向量。
 *
 * @param[in]  dev        Device handle from gs130_create().
 * @param[in]  from_frame Source frame.
 * @param[in]  to_frame   Target frame.
 * @param[out] T          Translation vector (3 doubles).
 * @return err code
 *
 * @note 当 from_frame == to_frame 时，理论上会返回零向量，但函数内部还是会进行矩阵运算，所以返回的零向量可能有精度差异。
 * @note 输出 T 向量的含义为：将 from_frame 坐标系下的点转换到 to_frame 坐标系下的平移向量，也就是以 to_frame 为参考系时 from_frame 的坐标。
 */
gs130_err_t gs130_get_relative_T(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *T);

/**
 * @brief 获取完整双目（IMU）标定参数
 *
 * @param[in]  dev         Device handle from gs130_create().
 * @param[out] calibration 双目（IMU）标定参数结构体
 * @return err code
 *
 * @note 注意不要混淆 gs130_calibration_t 中的外参含义，参考结构体定义处。
 */
gs130_err_t gs130_get_calibration(
    gs130_device_t *dev,
    gs130_calibration_t *calibration);

/**
 * @brief 变换设备的参考系
 *
 * 将指定的设备位姿变换为指定值，同时移动所有设备位姿，使得其他设备的位姿相对于该设备保持不变。
 *
 * @param[in] dev       Device handle from gs130_create().
 * @param[in] ref_frame 基准设备
 * @param[in] ref_R     基准设备的旋转矩阵 (9 doubles)
 * @param[in] ref_T     基准设备的平移向量 (3 doubles)
 * @return err code
 *
 * @note 本函数会修改设备内部保存的标定数据，后续 gs130_get_calibration / gs130_get_relative_R / gs130_get_relative_T 的返回值都会随之改变。
 * @note 仅允许在 gs130_init() 之后调用：init 会从 EEPROM 重新加载标定，此前的修改会被覆盖。本函数只修改标定数据，不会重新触发立体矫正计算。
 */
gs130_err_t gs130_convert_calibration(
    gs130_device_t *dev,
    gs130_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3]);

/**
 * @brief 获取探测到的 EEPROM 头声明
 *
 * @param[in] dev Device handle from gs130_create().
 * @return EEPROM 头，字符串；未检测到 EEPROM 时返回 NULL。
 */
const char *gs130_get_eeprom_name(
    gs130_device_t *dev);

/**
 * @brief 获取探测到的 EEPROM 详细信息（厂商、版本、畸变模型等）。
 *
 * @param[in] dev Device handle from gs130_create().
 * @return EEPROM 信息，字符串，未检测到 EEPROM 时返回 NULL。
 */
const char *gs130_get_eeprom_info(
    gs130_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* GS130_H */
