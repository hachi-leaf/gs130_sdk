/**
 * @file icm42688.cpp
 * @brief ICM-42688-P implementation.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/imu/imu.hpp"

#include <unistd.h>

#include <vector>

namespace gs130 {
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
constexpr uint8_t kIntfConfig5  = 0x7B;   // PIN9 muxed as FSYNC

constexpr uint8_t kPacketSize   = 16;
constexpr uint8_t kFsyncMask    = 0x0C;   // packet header bits[3:2]
constexpr uint8_t kFsyncVal     = 0x0C;   // 11 = FSYNC delta; 10 = normal timestamp

// delta_time tick conversion: us = raw * 32 / 30
constexpr uint32_t kTickNum = 32;
constexpr uint32_t kTickDen = 30;

// {physical value, register bits}. Keyed by physical value for easy line-by-line checking against the datasheet.
struct Code {
    uint16_t value;
    uint8_t  bits;
};

// GYRO/ACCEL_CONFIG0 low 4 bits
constexpr Code kOdr[] = {
    {200, 0x07},
    {500, 0x0F},
    {0, 0},
};

// Full-scale encoding is descending: 000 = +/-16g, 011 = +/-2g
constexpr Code kAccelFsr[] = {
    {16, 0x00},
    {8,  0x20},
    {4,  0x40},
    {2,  0x60},
    {0, 0},
};

// Also descending: 000 = +/-2000dps, 011 = +/-250dps
constexpr Code kGyroFsr[] = {
    {2000, 0x00},
    {1000, 0x20},
    {500,  0x40},
    {250,  0x60},
    {0, 0},
};

// Number of UI filter settings; see ModelDesc::info for setting-to-bandwidth mapping
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
    // Strict validation: unsupported values (including unset 0) always error out; no defaults, no rounding to nearest
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

    // Soft reset
    if(bus.update(kDeviceConfig, 0x01, 0x01) != Status::Ok)
        return Status::HwError;
    usleep(1500);

    // PIN9 muxed as FSYNC; this register is in bank 1
    if(!select_bank(bus, 1))
        return Status::HwError;
    if(bus.update(kIntfConfig5, 0x06, 0x02) != Status::Ok)
        return Status::HwError;
    if(!select_bank(bus, 0))
        return Status::HwError;

    if(bus.write(kIntfConfig1, 0x91) != Status::Ok)     // RC oscillator
        return Status::HwError;
    if(bus.write(kTmstConfig, 0x23) != Status::Ok)      // timestamp + delta mode
        return Status::HwError;
    if(bus.write(kFsyncConfig, 0x10) != Status::Ok)     // FSYNC rising edge
        return Status::HwError;
    if(bus.write(kFifoConfig1, 0x4F) != Status::Ok)     // TMST_FSYNC+TEMP+GYRO+ACCEL
        return Status::HwError;
    if(bus.write(kIntConfig1, 0x00) != Status::Ok)      // async reset off
        return Status::HwError;

    // Power on
    if(bus.update(kPwrMgmt0, 0x3F, 0x0F) != Status::Ok)
        return Status::HwError;
    usleep(50000);

    // FIFO to bypass first, then configure sampling parameters
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
    (void)bus.readBurst(kIntStatus, &int_status, 1);   // clear stale FIFO_FULL flag
    if(bus.update(kFifoConfig, 0xC0, 0x40) != Status::Ok)   // stream-to-FIFO
        return Status::HwError;
    return Status::Ok;
}

void stop(base::I2cDevice &bus)
{
    bus.update(kFifoConfig, 0xC0, 0x00);
}

Status read(base::I2cDevice &bus, ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out)
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

    std::vector<uint8_t> raw(static_cast<std::size_t>(pkts) * kPacketSize);
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

        // When the header is not FSYNC, this field is a normal timestamp, not an edge offset; set 0 to avoid misuse
        if(s.is_fsync){
            const uint32_t dt = static_cast<uint32_t>((p[14] << 8) | p[15]);
            s.delta_time_us = dt * kTickNum / kTickDen;
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
    return (val & 0x02) != 0;   // INT_STATUS bit1 = FIFO_FULL (auto-cleared on read)
}

void deinit(base::I2cDevice &bus)
{
    // Graceful shutdown: back to bank0 -> stop FIFO -> power down the sensor
    select_bank(bus, 0);
    bus.update(kFifoConfig, 0xC0, 0x00);
    bus.update(kPwrMgmt0, 0x3F, 0x00);
}

} // namespace

// The definition must use extern: a namespace-scope const has internal linkage by default in C++
extern const ModelDesc kIcm42688 = {
    kWhoAmI,
    0x47,
    "ICM-42688-P",
    "- odr:       200 | 500 Hz\n"
    "- accel fsr: 2 | 4 | 8 | 16 g\n"
    "- gyro fsr:  250 | 500 | 1000 | 2000 dps\n"
    "- bw_sel: 0..F\n"
    "  - 0 BW=ODR/2\n"
    "  - 1 BW=max(400Hz, ODR)/4\n"
    "  - 2 BW=max(400Hz, ODR)/5\n"
    "  - 3 BW=max(400Hz, ODR)/8\n"
    "  - 4 BW=max(400Hz, ODR)/10\n"
    "  - 5 BW=max(400Hz, ODR)/16\n"
    "  - 6 BW=max(400Hz, ODR)/20\n"
    "  - 7 BW=max(400Hz, ODR)/40\n"
    "  - 8 to 13: Reserved\n"
    "  - 14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)\n"
    "  - 15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)\n"
    "- noise density: accel 70 ug/rtHz, gyro 0.0028 dps/rtHz\n",
    init,
    start,
    stop,
    read,
    full,
    deinit,
};

GS130_IMU_REGISTER_MODEL(kIcm42688);

} // namespace imu
} // namespace gs130
