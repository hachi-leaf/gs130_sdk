/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef GS130_DEVICES_EEPROM_EEPROM_HPP
#define GS130_DEVICES_EEPROM_EEPROM_HPP

#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace eeprom {

// Model descriptor table: each model implements its own probe and parsing and is
// handed an I2C accessor already bound to a bus and address.
// Adding a model = write an implementation file + define a ModelDesc + register it in eeprom.cpp's table.
struct ModelDesc {
    const char *name;
    const char *info;

    // recognize this model's layout; header length and checksum algorithm vary
    // per model, so the model itself decides
    bool   (*probe)(base::I2cDevice &bus);
    Status (*read) (base::I2cDevice &bus, StereoImuModel *out);
};

// The constructor opens the bus and probes each model in turn; the I2C device
// handle is held for the object's lifetime (closed on destruction).
// Read-only device: no power-down or reset needed, so there is no deinit.
class Eeprom {
public:
    Eeprom(uint8_t bus, uint8_t addr);
    ~Eeprom() = default;

    // non-copyable
    Eeprom(const Eeprom &)            = delete;
    Eeprom &operator=(const Eeprom &) = delete;

    // bool conversion: true = a model is bound; false = probe failed
    explicit operator bool() const { return desc_ != nullptr; }

    Status read(StereoImuModel *out);

    const char *name() const;
    const char *info() const;

private:
    base::I2cDevice  bus_;
    const ModelDesc *desc_ = nullptr;
};

} // namespace eeprom
} // namespace gs130

#endif // GS130_DEVICES_EEPROM_EEPROM_HPP
