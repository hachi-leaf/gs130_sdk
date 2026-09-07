/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * UNION EEPROM：双目 + IMU、鱼眼、v1.2、旋转 0°、4 畸变系数。
 * 本文件自成一体：header、校验和、偏移、字段宽度全部局部化，不与其他型号共享代码。
 */
#include "devices/eeprom/eeprom.hpp"

#include <cstring>

namespace gs130 {
namespace eeprom {
namespace {

// ============================== 识别 ==============================
constexpr uint16_t kHeaderSize = 15;                 // header[0..14]
constexpr uint16_t kChecksumOff = kHeaderSize;       // 校验和紧随其后
constexpr uint8_t  kChecksumLen = 14;                // 只累加前 14 字节

constexpr uint8_t kHeader[kHeaderSize] = {
    0x55, 0x4e, 0x49, 0x4f, 0x4e, 0x00, 0x00, 0x00,  // "UNION"
    0x11, 0x01,   // 双目 + IMU
    0x01, 0x02,   // v1.2
    0x00,         // 旋转 0°
    0x04,         // 畸变系数个数 4
    0x00,
};

// ============================== 布局偏移 ==============================
// 相机内参与双目外参为 double(8B)，IMU 内参与 imu 外参为 float(4B)

constexpr uint16_t kLFx           = 0x0018;   // EEPROM 的 L 内参块起点
constexpr uint16_t kRFx           = 0x0081;   // EEPROM 的 R 内参块起点
constexpr uint16_t kStereoR       = 0x00EA;   // 双目旋转 3×3 (double)
constexpr uint16_t kStereoT       = 0x0132;   // 双目平移 3   (double)
constexpr uint16_t kAccelMisalign = 0x0153;
constexpr uint16_t kAccelScale    = 0x0177;
constexpr uint16_t kAccelBias     = 0x0183;
constexpr uint16_t kAccelNoise    = 0x018F;
constexpr uint16_t kAccelWalk     = 0x0193;
constexpr uint16_t kGyroMisalign  = 0x0197;
constexpr uint16_t kGyroScale     = 0x01BB;
constexpr uint16_t kGyroBias      = 0x01C7;
constexpr uint16_t kGyroNoise     = 0x01D3;
constexpr uint16_t kGyroWalk      = 0x01D7;
constexpr uint16_t kImuR          = 0x01E0;   // IMU → 左目 旋转 (float)
constexpr uint16_t kImuT          = 0x0204;   // IMU → 左目 平移 (float)
constexpr uint16_t kSize          = 0x0219;

double read_double(const uint8_t *buf, uint16_t off)
{
    double v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// EEPROM 里是 4 字节 float，加宽到 double 无损
double read_float(const uint8_t *buf, uint16_t off)
{
    float v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// 内参块：fx,fy,cx,cy 后接 4 个鱼眼畸变系数，均为 double
void read_intrinsics(const uint8_t *buf, CameraIntrinsics *cam, uint16_t off)
{
    cam->fx = read_double(buf, off + 0x00);
    cam->fy = read_double(buf, off + 0x08);
    cam->cx = read_double(buf, off + 0x10);
    cam->cy = read_double(buf, off + 0x18);

    cam->K[0] = cam->fx;
    cam->K[2] = cam->cx;
    cam->K[4] = cam->fy;
    cam->K[5] = cam->cy;
    cam->K[8] = 1.0f;

    cam->dist_model = DistModel::Fisheye;
    cam->dist_coeffs[0] = read_double(buf, off + 0x20);
    cam->dist_coeffs[1] = read_double(buf, off + 0x28);
    cam->dist_coeffs[2] = read_double(buf, off + 0x30);
    cam->dist_coeffs[3] = read_double(buf, off + 0x38);
}

// EEPROM 存的是 IMU→左目 的相对位姿，复合成 IMU→参考系再交给调用方
void compose_imu_to_ref(StereoImuModel *c, const double *imu_to_left_R,
                        const double *imu_to_left_T)
{
    for(uint8_t r = 0; r < 3; r++){
        for(uint8_t col = 0; col < 3; col++){
            c->imu_R[r * 3 + col] = 0.0;
            for(uint8_t k = 0; k < 3; k++)
                c->imu_R[r * 3 + col] += c->cam_left_R[r * 3 + k] * imu_to_left_R[k * 3 + col];
        }
        c->imu_T[r] = c->cam_left_T[r];
        for(uint8_t k = 0; k < 3; k++)
            c->imu_T[r] += c->cam_left_R[r * 3 + k] * imu_to_left_T[k];
    }
}

bool probe(base::I2cDevice &bus)
{
    // header 15B + 校验和 1B
    uint8_t buf[kHeaderSize + 1];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return false;

    // 校验和：前 14 字节求和，(sum % 255) + 1
    uint16_t sum = 0;
    for(uint8_t i = 0; i < kChecksumLen; i++)
        sum = static_cast<uint16_t>(sum + buf[i]);
    if(static_cast<uint8_t>((sum % 255) + 1) != buf[kChecksumOff])
        return false;

    return memcmp(buf, kHeader, kHeaderSize) == 0;
}

Status read(base::I2cDevice &bus, StereoImuModel *out)
{
    if(!out)
        return Status::ParamError;

    uint8_t buf[kSize];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return Status::HwError;

    *out = StereoImuModel{};

    // EEPROM 的 L/R 块直接对应模组左右目
    read_intrinsics(buf, &out->cam_left,  kLFx);
    read_intrinsics(buf, &out->cam_right, kRFx);

    out->install_angle = 0;

    // 本布局以右目为公共参考系，故 cam_right 为单位变换
    for(uint8_t i = 0; i < 9; i++){
        out->cam_right_R[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        out->cam_left_R[i]  = read_double(buf, kStereoR + i * 8);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->cam_right_T[i] = 0.0f;
        out->cam_left_T[i]  = read_double(buf, kStereoT + i * 8);
    }

    for(uint8_t i = 0; i < 9; i++){
        out->imu.accel_misalign[i] = read_float(buf, kAccelMisalign + i * 4);
        out->imu.gyro_misalign[i]  = read_float(buf, kGyroMisalign + i * 4);
    }
    for(uint8_t i = 0; i < 3; i++){
        out->imu.accel_scale[i] = read_float(buf, kAccelScale + i * 4);
        out->imu.accel_bias[i]  = read_float(buf, kAccelBias + i * 4);
        out->imu.gyro_scale[i]  = read_float(buf, kGyroScale + i * 4);
        out->imu.gyro_bias[i]   = read_float(buf, kGyroBias + i * 4);
    }
    out->imu.accel_noise       = read_float(buf, kAccelNoise);
    out->imu.accel_random_walk = read_float(buf, kAccelWalk);
    out->imu.gyro_noise        = read_float(buf, kGyroNoise);
    out->imu.gyro_random_walk  = read_float(buf, kGyroWalk);

    double imu_to_left_R[9];
    double imu_to_left_T[3];
    for(uint8_t i = 0; i < 9; i++)
        imu_to_left_R[i] = read_float(buf, kImuR + i * 4);
    for(uint8_t i = 0; i < 3; i++)
        imu_to_left_T[i] = read_float(buf, kImuT + i * 4);
    compose_imu_to_ref(out, imu_to_left_R, imu_to_left_T);

    return Status::Ok;
}

} // namespace

// 定义处必须写 extern：C++ 中 namespace 作用域的 const 默认是内部链接
extern const ModelDesc kUnionStereoImuFisheyeV1p2R0N4 = {
    "UNION Stereo-IMU Fisheye V1.2 Rotate-0 4-Distortion-parameters",
    "UNION v1.2 双目+IMU 鱼眼标定\n",
    probe,
    read,
};

} // namespace eeprom
} // namespace gs130
