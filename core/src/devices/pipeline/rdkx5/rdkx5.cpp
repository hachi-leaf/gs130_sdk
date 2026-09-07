/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * RDK X5 平台骨架：Pipeline 方法桩，待逐步填充。
 * 资源直接放在 Impl，调用节点时逐个传参，无共享上下文。
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

// 帧时间戳（ns）：优先用 trig_tv（LPWM 上升沿 = 曝光触发时刻），
// 拿不到再退回 timestamps / tv。
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
    bool probed     = false;   // 两路 sensor 探测成功
    bool inited = false;   // 已建流

    // 单路相机资源（调用节点时逐个传入）
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

    // 几何参数
    uint32_t input_w = 0, input_h = 0, output_w = 0, output_h = 0;
    uint32_t mid_w = 0, mid_h = 0, vse_chn = 0;
    int install_angle = 0;
};

Pipeline::Pipeline(
    uint8_t left_addr, uint8_t right_addr,
    const uint8_t *bus_list, size_t bus_num)
    : impl_(std::make_unique<Impl>())
{
    // 遍历 bus_list：读 chip id 确认该总线 + 地址上是 SC132GS
    for(size_t i = 0; i < bus_num; ++i){
        const uint8_t bus = bus_list[i];

        // 右目
        if(impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_addr == 0){
            base::I2cDevice dev(bus, right_addr);
            uint16_t id = 0;
            if(dev && dev.read16(kChipIdReg, &id) == Status::Ok && id == kChipId){
                impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_bus  = bus;
                impl_->cam[static_cast<size_t>(CamIndex::Right)].i2c_addr = right_addr;
            }
            dev.close();
        }
        // 左目
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

    // 几何参数
    impl_->input_w = cfg.sensor_width;
    impl_->input_h = cfg.sensor_height;
    impl_->output_w = cfg.output_width;
    impl_->output_h = cfg.output_height;

    if(cfg.mode == OutputMode::Raw)
        if(cfg.sensor_width != cfg.output_width || cfg.sensor_height != cfg.output_height)
            return Status::ParamError;

    // 确定 mipi_rx 和 reset_gpio
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        const int bus = impl_->cam[i].i2c_bus;
        if(bus < 0 || bus >= 32 || cfg.bus_mipi_rx[bus] == 0xFF)return Status::ParamError;

        impl_->cam[i].mipi_rx    = cfg.bus_mipi_rx[bus];
        impl_->cam[i].reset_gpio = cfg.bus_reset_gpio[bus];
    }

    // 建流
    // Camera 节点
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

    // VIN 节点
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

    // ISP 节点
    for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
        int ret = isp_open(
            &impl_->cam[i].isp,
            impl_->input_w, impl_->input_h);
        if(ret){
            deinit();
            return Status::HwError;
        }
    }

    // GDC 节点
    // Rect：rectify + 旋转；
    // Resize + 安装旋转：纯旋转；
    // Raw：跳过 GDC
    if(cfg.mode != OutputMode::Raw){
        if(cal == nullptr)return Status::ParamError;

        impl_->install_angle = ((cal->install_angle % 360) + 360) % 360;
        if(impl_->install_angle % 90 != 0) return Status::ParamError;

        // 处理 Rect 和 旋转 Resize 场景
        if(cfg.mode == OutputMode::Rect || impl_->install_angle != 0){
            // 旋转宽高
            const bool swap = (impl_->install_angle == 90 || impl_->install_angle == 270);
            const uint32_t src_w = swap ? impl_->input_h : impl_->input_w;
            const uint32_t src_h = swap ? impl_->input_w : impl_->input_h;

            // 生成 GDC Map
            std::vector<RemapPoint> map[static_cast<size_t>(CamIndex::Num)];
            // 双目畸变立体矫正
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
            // 旋转矫正
            else{
                impl_->mid_w = src_w;
                impl_->mid_h = src_h;

                // 生成恒等 Map
                const uint32_t n = impl_->mid_w * impl_->mid_h;
                for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
                    map[i].resize(n);
                    for(uint32_t p = 0; p < n; p++){
                        map[i][p].x = p % impl_->mid_w;
                        map[i][p].y = p / impl_->mid_w;
                    }
                }
            }

            // 每路 gdc_open（map[0]=Right，map[1]=Left）
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
        // 不过 GDC：mid 直接取 sensor 尺寸（VSE 的输入）
        else{
            impl_->mid_w = impl_->input_w;
            impl_->mid_h = impl_->input_h;
        }
    }

    // VSE 节点
    if(cfg.mode != OutputMode::Raw){
        // ROI 整除检查（防整数除法截断）
        if(roi_ratio_exact(impl_->mid_w, impl_->mid_h,
                           impl_->output_w, impl_->output_h) != 0){
            deinit();
            return Status::Unsupported;
        }

        // 等比取景 ROI + 升/降采样通道（0=降采样，5=升采样）
        const common_rect_t roi = aspect_roi(impl_->mid_w, impl_->mid_h,
                                             impl_->output_w, impl_->output_h);
        impl_->vse_chn = (impl_->output_w > roi.w || impl_->output_h > roi.h) ? 5 : 0;

        // 每路 vse_open
        for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++){
            int ret = vse_open(
                &impl_->cam[i].vse, impl_->mid_w, impl_->mid_h,
                impl_->output_w, impl_->output_h, impl_->vse_chn);
            if(ret){
                deinit();
                return Status::HwError;
            }
        }

        // 内参写回：VSE 等比裁剪 + 缩放（mid 坐标系 → output 坐标系）
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

    // 绑定节点并确定输出位置
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
        // teardown 是值传，句柄不会写回，这里清掉
        impl_->cam[i].cam_fd = 0;
        impl_->cam[i].vin = impl_->cam[i].isp = impl_->cam[i].vse = impl_->cam[i].gdc = 0;
        impl_->cam[i].vflow = 0;
        impl_->cam[i].output_node = 0;
        impl_->cam[i].output_chn  = 0;
    }

    // 再上电：vflow_destroy 已下电，恢复 sensor 到常规可探测态
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

    // 检测宽高，不符报错
    if((uint32_t)img.buffer.width != width || (uint32_t)img.buffer.height != height){
        hbn_vnode_releaseframe(c.output_node, c.output_chn, &img);
        return Status::ParamError;
    }

    // 逐行按各自 stride 拷贝
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
