/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * I2C Wrapper, No Thread-safe
 */
#ifndef GS130W_BASE_I2C_HPP
#define GS130W_BASE_I2C_HPP

#include <cstdint>

#include "types.hpp"

namespace gs130w {
namespace base {

class I2cDevice {
public:
    I2cDevice(uint8_t bus, uint8_t addr);
    ~I2cDevice();

    // 禁止 copy
    I2cDevice(const I2cDevice &)            = delete;
    I2cDevice &operator=(const I2cDevice &) = delete;

    // 允许 move
    I2cDevice(I2cDevice &&other) noexcept;
    I2cDevice &operator=(I2cDevice &&other) noexcept;

    // 布尔转换语义：true 成功；false 失败
    explicit operator bool() const { return fd_ >= 0; }

    bool is_open() const { return fd_ >= 0; }
    void close();
    Status read(uint8_t reg, uint8_t *val) const;
    Status read16(uint16_t reg, uint16_t *val) const;
    Status write(uint8_t reg, uint8_t val) const;
    Status update(uint8_t reg, uint8_t mask, uint8_t val) const;
    Status write16(uint16_t reg, uint16_t val) const;
    Status readBurst(uint8_t reg, uint8_t *buf, uint32_t len) const;
    Status readReg16(uint16_t reg, uint8_t *val) const;
    Status writeReg16(uint16_t reg, uint8_t val) const;
    Status readBurst16(uint16_t reg, uint8_t *buf, uint32_t len) const;

private:
    int fd_ = -1;
};

} // namespace base
} // namespace gs130w

#endif // GS130W_BASE_I2C_HPP