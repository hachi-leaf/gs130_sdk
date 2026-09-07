/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * ICM-42688-P 实现。基于 DS-000347 Rev 1.2。
 * 本文件自成一体：寄存器、编码、时序全部局部化，不与其他型号共享代码。
 */
#include "devices/imu/imu.hpp"

#include <unistd.h>
#include <cstdio>

#include <vector>

namespace gs130w {
namespace imu {
namespace {

// ---------------- bank 0 ----------------
constexpr uint8_t kBankSel      = 0x76;
constexpr uint8_t kDeviceConfig = 0x11;
constexpr uint8_t kFifoConfig   = 0x16;
constexpr uint8_t kFifoCount    = 0x2E;
constexpr uint8_t kIntStatus    = 0x2D;
constexpr uint8_t kFifoData     = 0x30;
constexpr uint8_t kIntfConfig1  = 0x4D;
constexpr uint8_t kPwrMgmt0     = 0x4E;
constexpr uint8_t kGyroConfig0  = 0x4F;
constexpr uint8_t kAccelConfig0 = 0x50;
constexpr uint8_t kGyroAccelCfg = 0x52;
constexpr uint8_t kTmstConfig   = 0x54;
constexpr uint8_t kFifoConfig1  = 0x5F;
constexpr uint8_t kFsyncConfig  = 0x62;
constexpr uint8_t kIntConfig1   = 0x64;
constexpr uint8_t kWhoAmI       = 0x75;

// ---------------- bank 1 ----------------
constexpr uint8_t kIntfConfig5  = 0x7B;   // PIN9 复用为 FSYNC

constexpr uint8_t kPacketSize   = 16;
constexpr uint8_t kFsyncMask    = 0x0C;   // 包头 bits[3:2]
constexpr uint8_t kFsyncVal     = 0x0C;   // 11 = FSYNC delta；10 = 普通时间戳

// delta_time 刻度换算：us = raw * 32 / 30
constexpr uint32_t kTickNum = 32;
constexpr uint32_t kTickDen = 30;

// {物理值, 寄存器位}。以物理值为键，便于与 datasheet 逐行核对。
struct Code {
    uint16_t value;
    uint8_t  bits;
};

// GYRO/ACCEL_CONFIG0 低 4 位
constexpr Code kOdr[] = {
    {200, 0x07},
    {500, 0x0F},
    {0, 0},
};

// 量程编码是降序的：000 = ±16g，011 = ±2g
constexpr Code kAccelFsr[] = {
    {16, 0x00},
    {8,  0x20},
    {4,  0x40},
    {2,  0x60},
    {0, 0},
};

// 同样降序：000 = ±2000dps，011 = ±250dps
constexpr Code kGyroFsr[] = {
    {2000, 0x00},
    {1000, 0x20},
    {500,  0x40},
    {250,  0x60},
    {0, 0},
};

// UI 滤波档位数，档位与带宽的对应关系见 ModelDesc::info
constexpr uint8_t kBwSelCount = 16;

bool lookup(const Code *tab, uint32_t value, uint8_t *bits)
{
    for(const Code *c = tab; c->value != 0; c++){
        if(c->value == value){
            *bits = c->bits;
            return true;
        }
    }
    return false;
}

bool select_bank(base::I2cDevice &bus, uint8_t bank)
{
    return bus.update(kBankSel, 0x07, bank) == Status::Ok;
}

Status init(base::I2cDevice &bus, const ImuConfig &cfg)
{
    // 严格校验：不支持的值（含未设置的 0）一律报错，不取默认值、不就近取整
    uint8_t odr_bits, afs_bits, gfs_bits;
    if(!lookup(kOdr, cfg.odr_hz, &odr_bits))                     return Status::Unsupported;
    if(!lookup(kAccelFsr, cfg.accel_fsr_g, &afs_bits))           return Status::Unsupported;
    if(!lookup(kGyroFsr, cfg.gyro_fsr_dps, &gfs_bits))           return Status::Unsupported;
    if(cfg.accel_bw_sel >= kBwSelCount)                          return Status::Unsupported;
    if(cfg.gyro_bw_sel  >= kBwSelCount)                          return Status::Unsupported;
    const uint8_t afilt = cfg.accel_bw_sel;
    const uint8_t gfilt = cfg.gyro_bw_sel;

    if(!select_bank(bus, 0))
        return Status::HwError;

    // 软复位
    if(bus.update(kDeviceConfig, 0x01, 0x01) != Status::Ok)
        return Status::HwError;
    usleep(1500);

    // PIN9 复用为 FSYNC，该寄存器在 bank 1
    if(!select_bank(bus, 1))
        return Status::HwError;
    if(bus.update(kIntfConfig5, 0x06, 0x02) != Status::Ok)
        return Status::HwError;
    if(!select_bank(bus, 0))
        return Status::HwError;

    if(bus.write(kIntfConfig1, 0x91) != Status::Ok)     // RC 振荡器
        return Status::HwError;
    if(bus.write(kTmstConfig, 0x23) != Status::Ok)      // 时间戳 + delta 模式
        return Status::HwError;
    if(bus.write(kFsyncConfig, 0x10) != Status::Ok)     // FSYNC 上升沿
        return Status::HwError;
    if(bus.write(kFifoConfig1, 0x4F) != Status::Ok)     // TMST_FSYNC+TEMP+GYRO+ACCEL
        return Status::HwError;
    if(bus.write(kIntConfig1, 0x00) != Status::Ok)      // async reset off
        return Status::HwError;

    // 上电
    if(bus.update(kPwrMgmt0, 0x3F, 0x0F) != Status::Ok)
        return Status::HwError;
    usleep(50000);

    // FIFO 先 bypass，再配采样参数
    if(bus.update(kFifoConfig, 0xC0, 0x00) != Status::Ok)
        return Status::HwError;
    if(bus.update(kGyroConfig0, 0xEF, static_cast<uint8_t>(gfs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.update(kAccelConfig0, 0xEF, static_cast<uint8_t>(afs_bits | odr_bits))
            != Status::Ok)
        return Status::HwError;
    if(bus.write(kGyroAccelCfg, static_cast<uint8_t>((afilt << 4) | gfilt))
            != Status::Ok)
        return Status::HwError;

    return Status::Ok;
}

Status start(base::I2cDevice &bus)
{
    uint8_t int_status;
    if(!select_bank(bus, 0))return Status::HwError;
    (void)bus.readBurst(kIntStatus, &int_status, 1);   // 清 FIFO_FULL 残留标志
    if(bus.update(kFifoConfig, 0xC0, 0x40) != Status::Ok)   // stream-to-FIFO
        return Status::HwError;
    return Status::Ok;
}

void stop(base::I2cDevice &bus)
{
    bus.update(kFifoConfig, 0xC0, 0x00);
}

Status read(base::I2cDevice &bus, ImuHwFifo16Packet *out, size_t cap, size_t *n_out)
{
    if(!out || !cap || !n_out)
        return Status::ParamError;
    *n_out = 0;

    uint8_t cbuf[2];
    if(bus.readBurst(kFifoCount, cbuf, 2) != Status::Ok)
        return Status::HwError;

    const uint16_t bytes = static_cast<uint16_t>((cbuf[0] << 8) | cbuf[1]);
    uint16_t pkts = static_cast<uint16_t>(bytes / kPacketSize);
    if(!pkts)
        return Status::Ok;
    if(pkts > cap)
        pkts = static_cast<uint16_t>(cap);

    std::vector<uint8_t> raw(static_cast<size_t>(pkts) * kPacketSize);
    if(bus.readBurst(kFifoData, raw.data(), static_cast<uint32_t>(raw.size()))
            != Status::Ok)
        return Status::HwError;

    for(uint16_t i = 0; i < pkts; i++){
        const uint8_t *p = raw.data() + i * kPacketSize;
        ImuHwFifo16Packet &s = out[i];

        s.is_fsync = (p[0] & kFsyncMask) == kFsyncVal;
        s.accel[0] = static_cast<int16_t>((p[1]  << 8) | p[2]);
        s.accel[1] = static_cast<int16_t>((p[3]  << 8) | p[4]);
        s.accel[2] = static_cast<int16_t>((p[5]  << 8) | p[6]);
        s.gyro[0]  = static_cast<int16_t>((p[7]  << 8) | p[8]);
        s.gyro[1]  = static_cast<int16_t>((p[9]  << 8) | p[10]);
        s.gyro[2]  = static_cast<int16_t>((p[11] << 8) | p[12]);
        s.temp     = static_cast<int8_t>(p[13]);

        // 包头非 FSYNC 时该字段是普通时间戳而非沿偏移，置 0 避免误用
        if(s.is_fsync){
            const uint32_t dt = static_cast<uint32_t>((p[14] << 8) | p[15]);
            s.delta_time_us = dt * kTickNum / kTickDen;
            fprintf(stderr, "[F] %u\n", s.delta_time_us);
        }
        else{
            s.delta_time_us = 0;
        }
    }

    *n_out = pkts;
    return Status::Ok;
}

bool full(base::I2cDevice &bus)
{
    uint8_t val = 0;
    if(!select_bank(bus, 0))return false;
    if(bus.readBurst(kIntStatus, &val, 1) != Status::Ok)return false;
    return (val & 0x01) != 0;   // INT_STATUS bit0 = FIFO_FULL（读后自动清）
}

void deinit(base::I2cDevice &bus)
{
    // 优雅退出：回 bank0 → 停 FIFO → 传感器下电
    select_bank(bus, 0);
    bus.update(kFifoConfig, 0xC0, 0x00);
    bus.update(kPwrMgmt0, 0x3F, 0x00);
}

} // namespace

// 定义处必须写 extern：C++ 中 namespace 作用域的 const 默认是内部链接
extern const ModelDesc kIcm42688 = {
    kWhoAmI,
    0x47,
    "ICM-42688-P",
    "ICM-42688-P\n"
    "  odr      : 200 / 500 Hz\n"
    "  accel fsr: 2 / 4 / 8 / 16 g\n"
    "  gyro fsr : 250 / 500 / 1000 / 2000 dps\n"
    "  bw_sel 0..7，带宽 = odr/2 (sel 0) 或 max(odr,400)/{4,5,8,10,16,20,40}:\n"
    "    bw_sel     0     1     2     3     4     5     6     7\n"
    "    odr 200    100   100   80    50    40    25    20    10   Hz\n"
    "    odr 500    250   125   100   62.5  50    31.25 25    12.5 Hz\n"
    "  噪声密度: accel 70 ug/rtHz, gyro 0.0028 dps/rtHz",
    init,
    start,
    stop,
    read,
    full,
    deinit,
};

} // namespace imu
} // namespace gs130w
