/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * RDK X5 platform skeleton: Pipeline method stubs, to be filled in step by step.
 * Resources live directly in Impl; passed one by one when calling nodes, no shared context.
 */
#include "devices/pipeline/rdkx5/rdkx5.h"

#include <cstdio>

#include "devices/pipeline/pipeline.hpp"

#include "base/i2c/i2c.hpp"
#include "base/rectify/rectify.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace gs130 {
namespace pipeline {

namespace {
// SC132GS chip id
constexpr uint16_t kChipIdReg = 0x3107;
constexpr uint16_t kChipId    = 0x0132;

// Frame timestamp (ns): prefer trig_tv (LPWM rising edge = exposure trigger time),
// fall back to timestamps / tv when unavailable.
uint64_t frame_ts_ns(const hbn_frame_info_t &info)
{
    if(info.trig_tv.tv_sec != 0 || info.trig_tv.tv_usec != 0)
        return static_cast<uint64_t>(info.trig_tv.tv_sec) * 1000000000ULL +
               static_cast<uint64_t>(info.trig_tv.tv_usec) * 1000ULL;
    if(info.timestamps != 0)
        return info.timestamps;
    return static_cast<uint64_t>(info.tv.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(info.tv.tv_usec) * 1000ULL;
}

} // namespace

struct Pipeline::Impl {
    bool probed     = false;   // both sensors probed successfully
    bool inited = false;   // stream built

    // Per-camera resources (passed one by one when calling nodes)
    struct CamHw {
        camera_handle_t    cam_fd  = 0;
        hbn_vnode_handle_t vin = 0, isp = 0, vse = 0, gdc = 0;
        hbn_vflow_handle_t vflow    = 0;
        hbn_vnode_handle_t output_node = 0;
        uint32_t           output_chn  = 0;
        hb_mem_common_buf_t gdc_bin{};
        int  mipi_rx = -1, reset_gpio = -1, i2c_bus = -1;
        uint8_t i2c_addr = 0;
    } cam[static_cast<size_t>(CamIndex::Num)];

    // Geometry parameters
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0, vse_chn = 0;
    int install_angle = 0;
};

Pipeline::Pipeline(
    uint8_t left_addr, uint8_t right_addr,
    const uint8_t *bus_list, size_t bus_num)
    : impl_(std::make_unique<Impl>())
{
    // Iterate bus_list: read chip id to confirm an SC132GS on that bus + address
    for(size_t i = 0; i < bus_num; ++i){
        const uint8_t bus = bus_list[i];

        // Right camera
        if(impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_addr == 0){
            base::I2cDevice dev(bus, right_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_bus  = bus;
                impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_addr = right_addr;
            }
            dev.close();
        }
        // Left camera
        if(impl_->cam[static_cast<size_t>(CamIndex::Left)].i2c_addr == 0){
            base::I2cDevice dev(bus, left_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<size_t>(CamIndex::Left)].i2c_bus  = bus;
                impl_->cam[static_cast<size_t>(CamIndex::Left)].i2c_addr = left_addr;
            }
            dev.close();
        }
    }

    impl_->probed = (impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_addr != 0 &&
                     impl_->cam[static_cast<size_t>(CamIndex::Left)].i2c_addr != 0);
}

Pipeline::~Pipeline()
{
    deinit();
}

Pipeline::operator bool() const { return impl_ && impl_->probed; }

Status Pipeline::init(const PipelineConfig &cfg, StereoImuModel *cal)
{
    if(!impl_->probed)return Status::NotFound;
    if(impl_->inited)return Status::ParamError;

    // Geometry parameters
    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height)
            return Status::ParamError;

    // Determine mipi_rx and reset_gpio
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF)return Status::ParamError;

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
    }

    // Build the stream
    // Camera node
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        int ret = camera_open(
            &impl_->cam[i].cam_fd,
            impl_->cam[i].i2c_addr,
            impl_->input_w, impl_->input_h, cfg.fps,
            cfg.line_length, cfg.frame_length,
            1200, 20, 1, cfg.tuning_file);

        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // VIN node
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        int ret = vin_open(
            &impl_->cam[i].vin,
            impl_->cam[i].mipi_rx,
            impl_->input_w, impl_->input_h, cfg.fps);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // ISP node
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        int ret = isp_open(
            &impl_->cam[i].isp,
            impl_->input_w, impl_->input_h);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // GDC node
    // Rect: rectify + rotation;
    // Resize + install rotation: pure rotation;
    // Raw: skip GDC
    if(cfg.mode != OutputMode::Raw){
        if(cal == nullptr)return Status::ParamError;

        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        // Handle Rect and rotated Resize cases
        if(cfg.mode == OutputMode::Rect || impl_->install_angle != 0){
            // Rotated width/height
            const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;

            // Generate GDC Map
            std::vector<RemapPoint> map[static_cast<size_t>(CamIndex::Num)];
            // Stereo distortion rectification for both cameras
            if(cfg.mode == OutputMode::Rect){
                Status st = base::stereo_rectify(cal, src_w, src_h,
                                                 &impl_->mid_w, &impl_->mid_h,
                                                 &map[static_cast<size_t>(CamIndex::Left)],
                                                 &map[static_cast<size_t>(CamIndex::Right)]);
                if(st != Status::Ok){
                    deinit();
                    return Status::Unsupported;
                }
            }
            // Rotation-only correction
            else{
                impl_->mid_w = src_w;
                impl_->mid_h = src_h;

                // Generate identity Map
                const uint32_t n = impl_->mid_w * impl_->mid_h;
                for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
                    map[i].resize(n);
                    for(uint32_t p = 0; p < n; p++){
                        map[i][p].x = p % impl_->mid_w;
                        map[i][p].y = p / impl_->mid_w;
                    }
                }
            }

            // gdc_open per camera (map[0]=Right, map[1]=Left)
            for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
                int ret = gdc_open(
                    &impl_->cam[i].gdc, &impl_->cam[i].gdc_bin,
                    map[i].data(), impl_->input_w, impl_->input_h,
                    impl_->mid_w, impl_->mid_h, impl_->install_angle);
                if(ret){
                    deinit();
                    return Status::HwError;
                }
            }
        }
        // Not through GDC: mid takes the sensor size directly (VSE input)
        else{
            impl_->mid_w = impl_->input_w;
            impl_->mid_h = impl_->input_h;
        }
    }

    // VSE node
    if(cfg.mode != OutputMode::Raw){
        // ROI divisibility check (guard against integer-division truncation)
        if(roi_ratio_exact(impl_->mid_w, impl_->mid_h,
                           impl_->output_w, impl_->output_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        // Aspect-ratio-preserving ROI + up/down-sampling channel (0=down, 5=up)
        const common_rect_t roi = aspect_roi(impl_->mid_w, impl_->mid_h,
                                             impl_->output_w, impl_->output_h);
        impl_->vse_chn = (impl_->output_w > roi.w || impl_->output_h > roi.h) ? 5 : 0;

        // vse_open per camera
        for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
            int ret = vse_open(
                &impl_->cam[i].vse, impl_->mid_w, impl_->mid_h,
                impl_->output_w, impl_->output_h, impl_->vse_chn);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        // Write back intrinsics: VSE aspect crop + scale (mid coords -> output coords)
        const double sfx = static_cast<double>(impl_->output_w) / roi.w;
        const double sfy = static_cast<double>(impl_->output_h) / roi.h;
        for(CameraIntrinsics *k : {&cal->cam_left, &cal->cam_right}){
            k->fx = k->fx * sfx;
            k->fy = k->fy * sfy;
            k->cx = (k->cx - roi.x) * sfx;
            k->cy = (k->cy - roi.y) * sfy;
            k->K[0] = k->fx; k->K[2] = k->cx;
            k->K[4] = k->fy; k->K[5] = k->cy;
        }
    }

    // Bind nodes and determine the output position
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        int ret = vflow_build(
            &impl_->cam[i].vflow, impl_->cam[i].cam_fd,
            impl_->cam[i].vin, impl_->cam[i].isp,
            impl_->cam[i].gdc, impl_->cam[i].vse,
            impl_->vse_chn,
            &impl_->cam[i].output_node, &impl_->cam[i].output_chn);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    impl_->inited = true;
    return Status::Ok;
}

void Pipeline::deinit()
{
    if(!impl_->inited)
        return;

    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        teardown_cam(impl_->cam[i].vflow, impl_->cam[i].cam_fd,
                     impl_->cam[i].vin, impl_->cam[i].isp,
                     impl_->cam[i].vse, impl_->cam[i].gdc,
                     &impl_->cam[i].gdc_bin, impl_->cam[i].reset_gpio);
        // teardown passes by value so handles are not written back; clear them here
        impl_->cam[i].cam_fd = 0;
        impl_->cam[i].vin = impl_->cam[i].isp = impl_->cam[i].vse = impl_->cam[i].gdc = 0;
        impl_->cam[i].vflow = 0;
        impl_->cam[i].output_node = 0;
        impl_->cam[i].output_chn  = 0;
    }

    // Power on again: vflow_destroy powered off; restore sensors to normal detectable state
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        if(impl_->cam[i].reset_gpio >= 0)
            sensor_power(impl_->cam[i].reset_gpio, 1);
    }

    impl_->inited = false;
}

Status Pipeline::start(CamIndex first)
{
    if(!impl_->inited)
        return Status::ParamError;

    const size_t a = static_cast<size_t>(first);
    const size_t b = static_cast<size_t>(
        first == CamIndex::Right ? CamIndex::Left : CamIndex::Right);

    if(hbn_vflow_start(impl_->cam[a].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    if(hbn_vflow_start(impl_->cam[b].vflow) != 0){
        deinit();
        return Status::HwError;
    }
    return Status::Ok;
}

void Pipeline::stop()
{
    if(!impl_->inited)
        return;

    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++)
        if(impl_->cam[i].vflow != 0)
            hbn_vflow_stop(impl_->cam[i].vflow);
}

Status Pipeline::get_frame(CamIndex idx,
                           uint8_t *y, uint8_t *uv,
                           uint32_t width, uint32_t height,
                           uint32_t y_stride, uint32_t uv_stride,
                           uint64_t *timestamp_ns,
                           uint32_t timeout_ms)
{
    if(y == nullptr || uv == nullptr || timestamp_ns == nullptr || !impl_->inited)
        return Status::ParamError;

    const size_t i = static_cast<size_t>(idx);
    auto &c = impl_->cam[i];

    hbn_vnode_image_t img;
    memset(&img, 0, sizeof(img));
    if(hbn_vnode_getframe(c.output_node, c.output_chn, timeout_ms, &img) != 0)
        return Status::Timeout;

    // Check width/height; report error on mismatch
    if((uint32_t)img.buffer.width != width || (uint32_t)img.buffer.height != height){
        hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    // Copy row by row using each plane's stride
    for(uint32_t r = 0; r < height; ++r)
        memcpy(y + (size_t)r * y_stride,
               img.buffer.virt_addr[0] + (size_t)r * img.buffer.stride, width);
    for(uint32_t r = 0; r < height / 2; ++r)
        memcpy(uv + (size_t)r * uv_stride,
               img.buffer.virt_addr[1] + (size_t)r * img.buffer.stride, width);

    fprintf(stderr, "[TV] frm=%u tv=%ld.%06ld trig=%ld.%06ld diff=%.3fms\n",
            img.info.frame_id,
            (long)img.info.tv.tv_sec, (long)img.info.tv.tv_usec,
            (long)img.info.trig_tv.tv_sec, (long)img.info.trig_tv.tv_usec,
            ((double)img.info.tv.tv_sec - img.info.trig_tv.tv_sec) * 1e3 +
            ((double)img.info.tv.tv_usec - img.info.trig_tv.tv_usec) / 1e3);
    *timestamp_ns = frame_ts_ns(img.info);

    hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
    return Status::Ok;
}

} // namespace pipeline
} // namespace gs130
