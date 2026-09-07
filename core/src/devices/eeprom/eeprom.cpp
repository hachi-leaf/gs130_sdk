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

namespace gs130 {
namespace eeprom {

// EEPROM model declarations
extern const ModelDesc kUnionStereoImuFisheyeV1p2R0N4;

namespace {

// EEPROM model registry table
const ModelDesc *const kModelTable[] = {
    &kUnionStereoImuFisheyeV1p2R0N4,
    nullptr,
};

} // namespace

Eeprom::Eeprom(uint8_t bus, uint8_t addr)
    : bus_(bus, addr)
{
    if(!bus_)return;

    for(const ModelDesc *const *m = kModelTable; *m; m++){
        if(!(*m)->probe(bus_))continue;
        desc_ = *m;
        return;
    }
}

Status Eeprom::read(StereoImuModel *out){return desc_->read(bus_, out);}

const char *Eeprom::name() const{return desc_->name;}

const char *Eeprom::info() const{return desc_->info;}

} // namespace eeprom
} // namespace gs130
