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

// 型号描述表：每个型号实现自己的一套操作，拿到的是已绑定总线与地址的 I2C 读写器。
// 新增型号 = 写一个实现文件 + 定义一份 ModelDesc + 登记到 imu.cpp 的表里。
struct ModelDesc {
    uint8_t who_am_i_addr;    // 探测用寄存器（须位于芯片默认 bank）
    uint8_t who_am_i_val;
    const char  *name;
    const char  *info;

    Status (*init)  (base::I2cDevice &bus, const ImuConfig &cfg);
    Status (*start) (base::I2cDevice &bus);
    void   (*stop)  (base::I2cDevice &bus);
    Status (*read)  (base::I2cDevice &bus, ImuHwFifo16Packet *out,
                     size_t cap, size_t *n_out);
    bool   (*full)(base::I2cDevice &bus);   // 读 INT_STATUS 检查 FIFO 满
    void   (*deinit)(base::I2cDevice &bus);   // 下电，由析构调用
};

// 构造时开总线并按 WHO_AM_I 匹配型号，持有该 I2C 设备句柄（析构即关闭）。
// 各方法原地转发到型号实现。
class Imu {
public:
    Imu(uint8_t bus, uint8_t addr);
    ~Imu();

    // 禁止 copy
    Imu(const Imu &)            = delete;
    Imu &operator=(const Imu &) = delete;

    // 布尔转换语义：true 已绑定型号；false 探测失败
    explicit operator bool() const { return desc_ != nullptr; }

    Status init(const ImuConfig &cfg);
    Status start();
    void   stop();

    // 读 FIFO；*n_out 返回实际包数
    Status read(ImuHwFifo16Packet *out, size_t cap, size_t *n_out);

    // 硬件 FIFO 是否已满（溢出丢包）
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
