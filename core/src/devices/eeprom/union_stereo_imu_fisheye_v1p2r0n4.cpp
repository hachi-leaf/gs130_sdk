/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * UNION EEPROM: stereo + IMU, fisheye, v1.2, rotation 0 deg, 4 distortion coefficients.
 * This file is self-contained: header, checksum, offsets, and field widths are all local; no code is shared with other models.
 */
#include "devices/eeprom/eeprom.hpp"

#include <cstring>

namespace gs130 {
namespace eeprom {
namespace {

// ============================== Identification ==============================
constexpr uint16_t kHeaderSize = 15;                 // header[0..14]
constexpr uint16_t kChecksumOff = kHeaderSize;       // checksum follows immediately
constexpr uint8_t  kChecksumLen = 14;                // only the first 14 bytes are summed

constexpr uint8_t kHeader[kHeaderSize] = {
    0x55, 0x4e, 0x49, 0x4f, 0x4e, 0x00, 0x00, 0x00,  // "UNION"
    0x11, 0x01,   // stereo + IMU
    0x01, 0x02,   // v1.2
    0x00,         // rotation 0 deg
    0x04,         // 4 distortion coefficients
    0x00,
};

// ============================== Layout offsets ==============================
// Camera intrinsics and stereo extrinsics are double (8B); IMU intrinsics and IMU extrinsics are float (4B)

constexpr uint16_t kLFx           = 0x0018;   // start of the EEPROM L intrinsics block
constexpr uint16_t kRFx           = 0x0081;   // start of the EEPROM R intrinsics block
constexpr uint16_t kStereoR       = 0x00EA;   // stereo rotation 3x3 (double)
constexpr uint16_t kStereoT       = 0x0132;   // stereo translation 3  (double)
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
constexpr uint16_t kImuR          = 0x01E0;   // IMU -> left camera rotation (float)
constexpr uint16_t kImuT          = 0x0204;   // IMU -> left camera translation (float)
constexpr uint16_t kSize          = 0x0219;

double read_double(const uint8_t *buf, uint16_t off)
{
    double v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// EEPROM stores 4-byte floats; widening to double is lossless
double read_float(const uint8_t *buf, uint16_t off)
{
    float v;
    memcpy(&v, &buf[off], sizeof(v));
    return v;
}

// Intrinsics block: fx,fy,cx,cy followed by 4 fisheye distortion coefficients, all double
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

// The EEPROM stores the relative pose IMU->left camera; compose it into IMU->reference frame before handing to the caller
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
    // header 15B + checksum 1B
    uint8_t buf[kHeaderSize + 1];
    if(bus.readBurst16(0x0000, buf, sizeof(buf)) != Status::Ok)
        return false;

    // Checksum: sum of the first 14 bytes, (sum % 255) + 1
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

    // The EEPROM L/R blocks map directly to the module's left/right cameras
    read_intrinsics(buf, &out->cam_left,  kLFx);
    read_intrinsics(buf, &out->cam_right, kRFx);

    out->install_angle = 0;

    // This layout uses the right camera as the common reference frame, so cam_right is the identity transform
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

// The definition must use extern: a namespace-scope const has internal linkage by default in C++
extern const ModelDesc kUnionStereoImuFisheyeV1p2R0N4 = {
    "UNION Stereo-IMU Fisheye V1.2 Rotate-0 4-Distortion-parameters",
    "UNION v1.2 stereo+IMU fisheye calibration\n",
    probe,
    read,
};

} // namespace eeprom
} // namespace gs130
