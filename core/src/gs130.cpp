/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * GS130 门面：把 Eeprom + Pipeline + Imu 封装成 C ABI。
 */
#include "gs130.h"

#include "types.hpp"

#include "devices/eeprom/eeprom.hpp"
#include "devices/imu/imu.hpp"
#include "devices/pipeline/pipeline.hpp"

#include "base/fifo/fifo.hpp"
#include "base/tracker/tracker.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace gs130;

// IMU 包缓存上限：超过说明 FSYNC 信号丢失
constexpr size_t kMaxImuCache = 1000;

/* 内部 Status → C 错误码 */
static gs130_err_t to_err(Status st)
{
    switch (st) {
    case Status::Ok:          return GS130_OK;
    case Status::ParamError:  return GS130_PARAM_ERROR;
    case Status::Unsupported: return GS130_UNSUPPORTED;
    case Status::NotFound:    return GS130_NOT_FOUND;
    case Status::HwError:     return GS130_HW_ERROR;
    case Status::Timeout:     return GS130_TIMEOUT;
    }
    return GS130_HW_ERROR;
}

/* 内部句柄 */
struct gs130_device_s {
    std::unique_ptr<eeprom::Eeprom>     eeprom;
    std::unique_ptr<imu::Imu>           imu;
    std::unique_ptr<pipeline::Pipeline> pipeline;

    // 数据队列：后台取帧/读 IMU 后入队，用户从队列取
    std::unique_ptr<base::Fifo<std::array<gs130_image_nv12_t, 2>>> camera_fifo;   // [CamIndex::Right]=0, [CamIndex::Left]=1
    std::unique_ptr<base::Fifo<gs130_imu_packet_t>> imu_fifo;

    // 标定缓存（EEPROM 读到后存这里，start 建流 / get_calibration 返回）
    StereoImuModel       cal_internal;   // 标定（get_calibration 时临时转 C 结构返回）

    // 线程
    std::thread             camera_thread;
    std::thread             imu_thread;
    std::atomic<bool>       running{false};
    std::atomic<bool>       camera_on{false};   // IMU 找到 FSYNC 后置 true，Camera 等它开流
    std::atomic<bool>       imu_fault{false};   // IMU 故障（FIFO 满 / FSYNC 丢失）

    // 时钟跟踪器（master=相机 LPWM 帧周期，slave=IMU FSYNC）
    std::unique_ptr<base::TimestampTracker> tracker;

    // 线程缓存：init 时从配置拷来
    uint32_t output_w = 0, output_h = 0;
    uint32_t fps = 0;
    uint16_t accel_fsr_g = 0, gyro_fsr_dps = 0;
    CamIndex fsync_camera = CamIndex::Right;   // IMU FSYNC 绑定的目（内部量）

    bool started = false;
};

/* ---- 后台线程 ---- */

// IMU 线程：读 FIFO → 读空后最新 FSYNC 置 camera_on → feed tracker → 修正时间戳入队
static void imu_thread_func(gs130_device_t *dev)
{
    const double accel_scale = (double)dev->accel_fsr_g * 9.80665 / 32768.0;
    const double gyro_scale  = (double)dev->gyro_fsr_dps * 3.14159265358979323846 / 180.0 / 32768.0;

    ImuHwFifo16Packet raw[128];   // 硬件 FIFO 容量 128 包
    std::vector<ImuHwFifo16Packet> cache;
    std::vector<gs130_imu_packet_t> pending;   // 弹出时间戳暂存，按旧到新入队
    const size_t max_cache = kMaxImuCache;

    bool available = false;

    while(dev->running.load()) {
        if(dev->imu->full()) { dev->imu_fault = true; break; }

        size_t n = 0;
        Status st = dev->imu->read(raw, 128, &n);
        if(st != Status::Ok || n == 0) { usleep(1000); continue; }

        // 遍历包
        for(size_t i = 0; i < n; i++){
            // 缓存太多 = 一直没有 FSYNC 锚点，报错
            if(cache.size() > max_cache){ dev->imu_fault = true; break; }

            // 尝试启动握手
            if(!available){
                // 最后一个包是 fsync 包
                if(i+1==n && raw[i].is_fsync){
                    // 握手完成
                    dev->camera_on.store(true);
                    available = true;
                }
            }
            // 握手后业务
            else{
                // feed 时间戳（FSYNC 包带沿偏移，普通包不带）
                const uint64_t delta_ns = static_cast<uint64_t>(raw[i].delta_time_us) * 1000;
                dev->tracker->feed_slave_sample(raw[i].is_fsync ? &delta_ns : nullptr);
                // 缓存 fifo 包
                cache.push_back(raw[i]);

                // 只在 FSYNC 锚点后配对（FSYNC 才往 ready_ 生成时间戳）
                if(raw[i].is_fsync){
                    // 取出全部时间戳（新到旧），配对 cache 尾部（新到旧），尽量匹配
                    const std::vector<uint64_t> ts_list = dev->tracker->take_ready();
                    fprintf(stderr, "[dbg-match] ready=%zu cache=%zu\n", ts_list.size(), cache.size());
                    size_t k = 0;
                    while(!cache.empty() && k < ts_list.size()){
                        const ImuHwFifo16Packet &p = cache.back();
                        gs130_imu_packet_t pkt;
                        pkt.accel[0] = (float)(p.accel[0] * accel_scale);
                        pkt.accel[1] = (float)(p.accel[1] * accel_scale);
                        pkt.accel[2] = (float)(p.accel[2] * accel_scale);
                        pkt.gyro[0]  = (float)(p.gyro[0] * gyro_scale);
                        pkt.gyro[1]  = (float)(p.gyro[1] * gyro_scale);
                        pkt.gyro[2]  = (float)(p.gyro[2] * gyro_scale);
                        pkt.temp     = (float)p.temp / 2.07f + 25.0f;
                        pkt.is_fsync = p.is_fsync;
                        pkt.timestamp_ns = ts_list[k];
                        pending.push_back(pkt);
                        cache.pop_back();
                        k++;
                    }
                    // 哪边先消耗完，就把两边剩余一起清空（cache 剩余丢弃；ts_list 局部变量自动释放）
                    cache.clear();
                    // 发布：按旧到新 push 进 imu_fifo
                    for(auto it = pending.rbegin(); it != pending.rend(); ++it)
                        dev->imu_fifo->push(*it);
                    pending.clear();
                }

            }
        }
        if(dev->imu_fault)break;
    }
}

// Camera 线程：等 camera_on → 开流 → 取双目对齐 → 每帧上传相位 → 入队
static void camera_thread_func(gs130_device_t *dev)
{
    while(dev->running.load() && !dev->camera_on.load())usleep(1000);
    if(!dev->running.load())return;
    const Status st_start = dev->pipeline->start(dev->fsync_camera);
    if(st_start != Status::Ok){ dev->running.store(false); return; }
    const uint32_t w = dev->output_w, h = dev->output_h, stride = w;
    const uint64_t tol = 1000000000ULL / dev->fps / 2;
    const size_t R = static_cast<size_t>(CamIndex::Right);
    const size_t L = static_cast<size_t>(CamIndex::Left);
    uint64_t idx = 0;         // 帧序号，自增

    while(dev->running.load()) {
        std::array<gs130_image_nv12_t, 2> f{};
        Status st;
        // 先取 FSYNC 绑定目（主时钟），再取另一目
        const CamIndex F = dev->fsync_camera;
        const CamIndex O = (F == CamIndex::Right) ? CamIndex::Left : CamIndex::Right;
        const size_t fi = static_cast<size_t>(F);
        const size_t oi = static_cast<size_t>(O);
        for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++) {
            f[i].y  = (uint8_t *)malloc((size_t)h * stride);
            f[i].uv = (uint8_t *)malloc((size_t)h / 2 * stride);
            if(!f[i].y || !f[i].uv)goto free_frame;
            f[i].width = w; f[i].height = h;
            f[i].y_stride = stride; f[i].uv_stride = stride;
        }

        st = dev->pipeline->get_frame(F, f[fi].y, f[fi].uv,
                                      w, h, stride, stride, &f[fi].timestamp_ns, 100);
        if(st != Status::Ok)goto free_frame;
        // 绑定目新帧先上传主时钟相位
        dev->tracker->update_master_timestamp_ns(f[fi].timestamp_ns);

        st = dev->pipeline->get_frame(O, f[oi].y, f[oi].uv,
                                      w, h, stride, stride, &f[oi].timestamp_ns, 100);
        if(st != Status::Ok)goto free_frame;

        // 半帧范围对齐：丢时间戳小（旧）的那目，重取，直到对齐
        for(;;) {
            uint64_t tR = f[R].timestamp_ns, tL = f[L].timestamp_ns;
            uint64_t d = (tR > tL) ? (tR - tL) : (tL - tR);
            if(d <= tol)break;
            if(!dev->running.load())goto free_frame;

            const size_t lag = (tR < tL) ? R : L;   // 时间戳小 = 旧帧
            const CamIndex lag_idx = (lag == R) ? CamIndex::Right : CamIndex::Left;
            st = dev->pipeline->get_frame(lag_idx, f[lag].y, f[lag].uv,
                                          w, h, stride, stride, &f[lag].timestamp_ns, 100);
            if(st != Status::Ok)goto free_frame;
            if(lag == fi)   // 重取的是绑定目，再上传一次相位
                dev->tracker->update_master_timestamp_ns(f[fi].timestamp_ns);
        }

        if(!dev->camera_fifo->push(f))goto free_frame;
        idx++;
        continue;

free_frame:
        for(size_t i = 0; i < static_cast<size_t>(CamIndex::Num); i++) { free(f[i].y); free(f[i].uv); }
    }
}

extern "C" {

gs130_device_t *gs130_create()
{
    return new gs130_device_t;
}

void gs130_destroy(gs130_device_t *dev)
{
    delete dev;
}


gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg)
{
    if(dev == nullptr || cfg == nullptr)return GS130_PARAM_ERROR;
    if(dev->eeprom || dev->pipeline || dev->imu)return GS130_PARAM_ERROR;   // 已 init

    // 创建 EEPROM 对象：循环候选总线探测，失败置空
    for(size_t i = 0; i < cfg->eeprom_config.bus_num; i++){
        dev->eeprom.reset(
            new eeprom::Eeprom(cfg->eeprom_config.bus[i], cfg->eeprom_config.addr));
        if(*dev->eeprom)break;
    }
    if(dev->eeprom && !*dev->eeprom)dev->eeprom.reset();

    // 创建 IMU 对象：循环候选总线探测，失败置空
    for(size_t i = 0; i < cfg->imu_config.bus_num; i++) {
        dev->imu.reset(
            new imu::Imu(cfg->imu_config.bus[i], cfg->imu_config.addr));
        if(*dev->imu)break;
    }
    if(dev->imu && !*dev->imu)dev->imu.reset();

    // 创建 Pipeline 对象
    dev->pipeline.reset(
        new pipeline::Pipeline(
            cfg->camera_config.left_addr, cfg->camera_config.right_addr,
            cfg->camera_config.bus, cfg->camera_config.bus_num));
    if(!*dev->pipeline)return GS130_NOT_FOUND;

    // 创建 FIFO（depth < 2 时 Fifo 构造会自动置无效）
    dev->camera_fifo.reset(new base::Fifo<std::array<gs130_image_nv12_t, 2>>(
        cfg->camera_fifo.depth, static_cast<FifoMode>(cfg->camera_fifo.mode)));
    dev->imu_fifo.reset(new base::Fifo<gs130_imu_packet_t>(
        cfg->imu_fifo.depth, static_cast<FifoMode>(cfg->imu_fifo.mode)));

    // 读取 EEPROM 标定（探测成功则读，失败映射错误类型）
    if(dev->eeprom) {
        Status st = dev->eeprom->read(&dev->cal_internal);
        if(st != Status::Ok)return to_err(st);
    }

    // 初始化 IMU（配置，不开流；探测到了就必须配成功）
    if(dev->imu) {
        ImuConfig ic{};
        ic.odr_hz       = cfg->imu_config.odr_hz;
        ic.accel_fsr_g  = cfg->imu_config.accel_fsr_g;
        ic.gyro_fsr_dps = cfg->imu_config.gyro_fsr_dps;
        ic.accel_bw_sel = cfg->imu_config.accel_bw_sel;
        ic.gyro_bw_sel  = cfg->imu_config.gyro_bw_sel;
        Status st = dev->imu->init(ic);
        if(st != Status::Ok)return to_err(st);
    }

    // 缓存线程所需参数
    dev->output_w = cfg->camera_config.output_width;
    dev->output_h = cfg->camera_config.output_height;
    dev->fps      = cfg->camera_config.fps;
    dev->accel_fsr_g  = cfg->imu_config.accel_fsr_g;
    dev->gyro_fsr_dps = cfg->imu_config.gyro_fsr_dps;
    dev->fsync_camera = (cfg->camera_config.fsync_camera == GS130_CAMERA_RIGHT_IDX)
                             ? CamIndex::Right : CamIndex::Left;

    // 时钟跟踪器（master=相机 LPWM 帧周期，slave=IMU FSYNC）
    dev->tracker.reset(new base::TimestampTracker(1000000000ULL / dev->fps));

    // 初始化 Camera（建流配置，不开流）
    {
        const gs130_camera_config_t &cc = cfg->camera_config;
        PipelineConfig pc{};
        memcpy(pc.bus_mipi_rx,    cc.bus_mipi_rx,    sizeof(pc.bus_mipi_rx));
        memcpy(pc.bus_reset_gpio, cc.bus_reset_gpio, sizeof(pc.bus_reset_gpio));
        pc.sensor_width   = cc.sensor_width;
        pc.sensor_height  = cc.sensor_height;
        pc.fps            = cc.fps;
        pc.line_length    = cc.line_length;
        pc.frame_length   = cc.frame_length;
        pc.tuning_file    = cc.tuning_file;
        pc.mode           = static_cast<OutputMode>(cc.mode);
        pc.output_width   = cc.output_width;
        pc.output_height  = cc.output_height;

        StereoImuModel *cal = dev->eeprom ? &dev->cal_internal : nullptr;
        if(cal == nullptr && cc.mode != GS130_CAMERA_MODE_RAW)return GS130_PARAM_ERROR;
        Status st = dev->pipeline->init(pc, cal);
        if(st != Status::Ok)return to_err(st);
    }

    return GS130_OK;
}

gs130_err_t gs130_start(gs130_device_t *dev)
{
    if(dev == nullptr)return GS130_PARAM_ERROR;
    if(dev->started)return GS130_PARAM_ERROR;
    if(!dev->pipeline)return GS130_PARAM_ERROR;

    dev->camera_on = false;
    dev->running = true;

    if(dev->imu) {
        Status st = dev->imu->start();
        if(st != Status::Ok) { dev->running = false; return to_err(st); }
        dev->imu_thread = std::thread(imu_thread_func, dev);
    } else {
        dev->camera_on = true;   // 无 IMU：不握手，直接开流
    }
    dev->camera_thread = std::thread(camera_thread_func, dev);

    dev->started = true;
    return GS130_OK;
}

void gs130_stop(gs130_device_t *dev)
{
    if(dev == nullptr || !dev->started)return;
    dev->running = false;
    if(dev->camera_thread.joinable())dev->camera_thread.join();
    if(dev->imu_thread.joinable())dev->imu_thread.join();
    dev->pipeline->stop();
    if(dev->imu)dev->imu->stop();
    dev->started = false;
}

gs130_err_t gs130_deinit(gs130_device_t *dev)
{
    if(dev == nullptr)return GS130_PARAM_ERROR;

    if(dev->started)gs130_stop(dev);

    dev->pipeline.reset();   // 触发析构：拆流 + sensor 恢复上电
    dev->imu.reset();        // 关闭 I2C
    dev->eeprom.reset();     // 关闭 I2C
    dev->camera_fifo.reset();
    dev->imu_fifo.reset();
    return GS130_OK;
}

/* ---- 相机数据 ---- */

size_t gs130_available_camera(gs130_device_t *dev)
{
    if(dev == nullptr || !dev->camera_fifo)return 0;
    return dev->camera_fifo->size();
}

gs130_err_t gs130_get_nv12_frame(gs130_device_t *dev,
                                   gs130_image_nv12_t *image_left,
                                   gs130_image_nv12_t *image_right)
{
    if(dev == nullptr || image_left == nullptr || image_right == nullptr)
        return GS130_PARAM_ERROR;
    if(!dev->camera_fifo)return GS130_PARAM_ERROR;

    std::array<gs130_image_nv12_t, 2> f;
    if(!dev->camera_fifo->pop(f))return GS130_TIMEOUT;

    const size_t R = static_cast<size_t>(CamIndex::Right);
    const size_t L = static_cast<size_t>(CamIndex::Left);
    *image_left  = f[L];
    *image_right = f[R];
    return GS130_OK;
}

/* ---- IMU 数据 ---- */

size_t gs130_available_imu(gs130_device_t *dev)
{
    if(dev == nullptr || !dev->imu || !dev->imu_fifo)return 0;
    if(dev->imu_fault.load())return 0;
    return dev->imu_fifo->size();
}

gs130_err_t gs130_read_imu(gs130_device_t *dev,
                             gs130_imu_packet_t *out)
{
    if(dev == nullptr || out == nullptr)return GS130_PARAM_ERROR;
    if(!dev->imu || !dev->imu_fifo)return GS130_PARAM_ERROR;
    if(dev->imu_fault.load())return GS130_HW_ERROR;
    if(!dev->imu_fifo->pop(*out))return GS130_TIMEOUT;
    return GS130_OK;
}

/* ---- 标定 ---- */

gs130_err_t gs130_get_calibration(gs130_device_t *dev,
                                    gs130_calibration_t *calibration)
{
    if(dev == nullptr || calibration == nullptr)return GS130_PARAM_ERROR;
    if(!dev->eeprom)return GS130_NOT_FOUND;
    memcpy(calibration, &dev->cal_internal, sizeof(*calibration));
    return GS130_OK;
}


} // extern "C"
