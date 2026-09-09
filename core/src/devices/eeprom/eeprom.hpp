/**
 * @file eeprom.hpp
 * @brief EEPROM binocular (IMU) distortion - stereo calibration parameter reader.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#ifndef GS130_DEVICES_EEPROM_EEPROM_HPP
#define GS130_DEVICES_EEPROM_EEPROM_HPP

#include <cstdint>

#include "base/i2c/i2c.hpp"
#include "types.hpp"

namespace gs130 {
namespace eeprom {

struct ModelDesc{
    const char *name;
    const char *info;

    bool   (*probe)(base::I2cDevice &bus);
    Status (*read) (base::I2cDevice &bus, StereoImuModel *out);
};

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

// Self-registration: a model file calls GS130_EEPROM_REGISTER_MODEL(<desc>) at file
// scope; the registry fills at static-init time, so a new model file needs no edit here.
bool register_model(const ModelDesc *desc);

} // namespace eeprom
} // namespace gs130

// Self-registration macro: call at file scope in a model .cpp, after the ModelDesc definition.
#define GS130_EEPROM_REGISTER_MODEL(DESC) \
    namespace { [[maybe_unused]] const bool gs130_reg_##DESC = ::gs130::eeprom::register_model(&DESC); }

#endif // GS130_DEVICES_EEPROM_EEPROM_HPP
