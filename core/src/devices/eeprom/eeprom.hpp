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

// 型号描述表：每个型号实现自己的探测与解析，拿到的是已绑定总线与地址的 I2C 读写器。
// 新增型号 = 写一个实现文件 + 定义一份 ModelDesc + 登记到 eeprom.cpp 的表里。
struct ModelDesc {
    const char *name;
    const char *info;

    // 识别本型号布局；header 长度、校验和算法因型号而异，故由型号自己判定
    bool   (*probe)(base::I2cDevice &bus);
    Status (*read) (base::I2cDevice &bus, StereoImuModel *out);
};

// 构造时开总线并逐个型号探测，持有该 I2C 设备句柄（析构即关闭）。
// 只读器件，无需下电或复位，故没有 deinit。
class Eeprom {
public:
    Eeprom(uint8_t bus, uint8_t addr);
    ~Eeprom() = default;

    // 禁止 copy
    Eeprom(const Eeprom &)            = delete;
    Eeprom &operator=(const Eeprom &) = delete;

    // 布尔转换语义：true 已绑定型号；false 探测失败
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
