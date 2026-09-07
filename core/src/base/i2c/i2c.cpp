/**
 * @file i2c.cpp
 * @brief I2cDevice implementation: plain i2c-dev read/write transactions.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/i2c/i2c.hpp"

#include <cstdio>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

namespace gs130 {
namespace base {

I2cDevice::~I2cDevice()
{
    close();
}

I2cDevice::I2cDevice(I2cDevice &&other) noexcept
    : fd_(other.fd_)
{
    other.fd_ = -1;
}

I2cDevice &I2cDevice::operator=(I2cDevice &&other) noexcept
{
    if(this != &other){
        close(); // release this object's old fd first
        fd_ = other.fd_; // take over the other's fd
        other.fd_ = -1;
    }
    return *this;
}

I2cDevice::I2cDevice(uint8_t bus, uint8_t addr)
{
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%u", bus);

    fd_ = ::open(path, O_RDWR);
    if(fd_ < 0)return ; // failure: object invalid (!*this)

    if(::ioctl(fd_, I2C_SLAVE_FORCE, addr) < 0){
        ::close(fd_);
        fd_ = -1;
    }
}

void I2cDevice::close()
{
    if(fd_ >= 0){
        ::close(fd_);
        fd_ = -1;
    }
}

Status I2cDevice::read(uint8_t reg, uint8_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;
    if(::write(fd_, &reg, 1) != 1)return Status::HwError;
    if(::read(fd_, val, 1) != 1)return Status::HwError;

    return Status::Ok;
}

Status I2cDevice::read16(uint16_t reg, uint16_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
    };
    uint8_t rbuf[2];

    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, rbuf, 2) != 2)return Status::HwError;

    *val = static_cast<uint16_t>((rbuf[0] << 8) | rbuf[1]);

    return Status::Ok;
}

Status I2cDevice::write(uint8_t reg, uint8_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[2] = {reg, val};
    if(::write(fd_, buf, 2) != 2)return Status::HwError;
    
    return Status::Ok;
}

Status I2cDevice::update(uint8_t reg, uint8_t mask, uint8_t val) const
{
    uint8_t old_val = 0;
    Status ret = read(reg, &old_val);
    if(ret != Status::Ok)return ret;

    const uint8_t new_val = static_cast<uint8_t>(
        (old_val & static_cast<uint8_t>(~mask)) |
        (val & mask));

    if(new_val == old_val)return Status::Ok;

    return write(reg, new_val);
}

Status I2cDevice::write16(uint16_t reg, uint16_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[4] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        static_cast<uint8_t>(val >> 8),
        static_cast<uint8_t>(val & 0xFF)};

    if(::write(fd_, buf, 4) != 4)return Status::HwError;
    
    return Status::Ok;
}

Status I2cDevice::readBurst(uint8_t reg, uint8_t *buf, uint32_t len) const
{
    if(fd_ < 0 || !buf || !len)return Status::ParamError;
    if(::write(fd_, &reg, 1) != 1)return Status::HwError;
    if(::read(fd_, buf, len) != static_cast<ssize_t>(len))
        return Status::HwError;
    
    return Status::Ok;
}

Status I2cDevice::readReg16(uint16_t reg, uint8_t *val) const
{
    if(fd_ < 0 || !val)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)};
    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, val, 1) != 1)return Status::HwError;
    
    return Status::Ok;
}

Status I2cDevice::writeReg16(uint16_t reg, uint8_t val) const
{
    if(fd_ < 0)return Status::ParamError;

    const uint8_t buf[3] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF),
        val};
    
    if(::write(fd_, buf, 3) != 3)return Status::HwError;
    
    return Status::Ok;
}

Status I2cDevice::readBurst16(uint16_t reg, uint8_t *buf, uint32_t len) const
{
    if(fd_ < 0 || !buf || !len)return Status::ParamError;

    const uint8_t addr[2] = {
        static_cast<uint8_t>(reg >> 8),
        static_cast<uint8_t>(reg & 0xFF)};

    if(::write(fd_, addr, 2) != 2)return Status::HwError;
    if(::read(fd_, buf, len) != static_cast<ssize_t>(len))
        return Status::HwError;
    
    return Status::Ok;
}

} // namespace base
} // namespace gs130