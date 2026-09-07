/**
 * @file imu.hpp
 * @brief General Packaging for IMU Devices.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEVICES_IMU_IMU_HPP
#define GS130_DEVICES_IMU_IMU_HPP

#include <cstddef>
#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace imu {

struct ModelDesc {
    uint8_t who_am_i_addr;
    uint8_t who_am_i_val;
    const char  *name;
    const char  *info;

    Status (*init)  (base::I2cDevice &bus, const ImuConfig &cfg);
    Status (*start) (base::I2cDevice &bus);
    void   (*stop)  (base::I2cDevice &bus);
    Status (*read)  (base::I2cDevice &bus, ImuHwFifo16Packet *out,
                     std::size_t cap, std::size_t *n_out);
    bool   (*full)(base::I2cDevice &bus);
    void   (*deinit)(base::I2cDevice &bus);
};

class Imu {
public:
    Imu(uint8_t bus, uint8_t addr);
    ~Imu();

    // non-copyable
    Imu(const Imu &)            = delete;
    Imu &operator=(const Imu &) = delete;

    // bool conversion: true = a model is bound; false = probe failed
    explicit operator bool() const { return desc_ != nullptr; }

    Status init(const ImuConfig &cfg);
    Status start();
    void   stop();

    // read FIFO; *n_out returns the actual packet count
    Status read(ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out);

    // whether the hardware FIFO is full (overflow drops packets)
    bool full();

    const char *name() const;
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

} // namespace imu
} // namespace gs130

#endif // GS130_DEVICES_IMU_IMU_HPP
