/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef GS130_DEVICES_IMU_IMU_HPP
#define GS130_DEVICES_IMU_IMU_HPP

#include <cstddef>
#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace imu {

// Model descriptor table: each model implements its own set of operations and is
// handed an I2C accessor already bound to a bus and address.
// Adding a model = write an implementation file + define a ModelDesc + register it in imu.cpp's table.
struct ModelDesc {
    uint8_t who_am_i_addr;    // probe register (must be in the chip's default bank)
    uint8_t who_am_i_val;
    const char  *name;
    const char  *info;

    Status (*init)  (base::I2cDevice &bus, const ImuConfig &cfg);
    Status (*start) (base::I2cDevice &bus);
    void   (*stop)  (base::I2cDevice &bus);
    Status (*read)  (base::I2cDevice &bus, ImuHwFifo16Packet *out,
                     size_t cap, size_t *n_out);
    bool   (*full)(base::I2cDevice &bus);   // read INT_STATUS to check FIFO full
    void   (*deinit)(base::I2cDevice &bus);   // power down, called from the destructor
};

// The constructor opens the bus and matches the model by WHO_AM_I; the I2C device
// handle is held for the object's lifetime (closed on destruction).
// Each method forwards directly to the model implementation.
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
    Status read(ImuHwFifo16Packet *out, size_t cap, size_t *n_out);

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
