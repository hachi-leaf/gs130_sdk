/**
 * @file eeprom.cpp
 * @brief EEPROM model registry table and probe loop.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/eeprom/eeprom.hpp"

#include <vector>

namespace gs130 {
namespace eeprom {

namespace {

// Auto-populated model registry: each model file self-registers via
// GS130_EEPROM_REGISTER_MODEL at static-init time.
std::vector<const ModelDesc *> &registry()
{
    static std::vector<const ModelDesc *> r;   // function-local static: safe init order
    return r;
}

} // namespace

bool register_model(const ModelDesc *desc)
{
    registry().push_back(desc);
    return true;
}

Eeprom::Eeprom(uint8_t bus, uint8_t addr)
    : bus_(bus, addr)
{
    if(!bus_)return;

    for(const ModelDesc *m : registry()){
        if(!m->probe(bus_))continue;
        desc_ = m;
        return;
    }
}

Status Eeprom::read(StereoImuModel *out){return desc_->read(bus_, out);}

const char *Eeprom::name() const{return desc_->name;}

const char *Eeprom::info() const{return desc_->info;}

} // namespace eeprom
} // namespace gs130
