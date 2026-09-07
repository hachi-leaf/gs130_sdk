/**
 * @file imu.cpp
 * @brief IMU model registry table and probe loop.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/imu/imu.hpp"

namespace gs130 {
namespace imu {

extern const ModelDesc kIcm42688;

namespace {

const ModelDesc *const kModelTable[] = {
    &kIcm42688,
    nullptr,
};

} // namespace

Imu::Imu(uint8_t bus, uint8_t addr)
    : bus_(bus, addr)
{
    if(!bus_)return;

    for(const ModelDesc *const *m = kModelTable; *m; m++){
        uint8_t val = 0;
        if(bus_.read((*m)->who_am_i_addr, &val) != Status::Ok)continue;
        if(val != (*m)->who_am_i_val)continue;

        desc_ = *m;
        return;
    }
}

Imu::~Imu()
{
    if(desc_ && bus_)desc_->deinit(bus_);
}

Status Imu::init(const ImuConfig &cfg){return desc_->init(bus_, cfg);}

Status Imu::start(){return desc_->start(bus_);}

void Imu::stop(){desc_->stop(bus_);}

Status Imu::read(ImuHwFifo16Packet *out, std::size_t cap, std::size_t *n_out){return desc_->read(bus_, out, cap, n_out);}

bool Imu::full(){return desc_ && desc_->full(bus_);}

const char *Imu::name() const{return desc_->name;}

const char *Imu::info() const{return desc_->info;}

} // namespace imu
} // namespace gs130
