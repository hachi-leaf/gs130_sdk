/**
 * @file gs130.cpp
 * @brief Encapsulates the IMU, stereo camera and EEPROM devices behind the public header (gs130.h).
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"

#include "types.hpp"

#include "base/fifo/fifo.hpp"
#include "base/tracker/tracker.hpp"

#include "devices/eeprom/eeprom.hpp"
#include "devices/imu/imu.hpp"
#include "devices/pipeline/pipeline.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

using namespace gs130;

/* Internal Status -> C error code */
static gs130_err_t to_err(Status st)
{
    switch (st) {
    case Status::Ok:          return GS130_OK;
    case Status::ParamError:  return GS130_PARAM_ERROR;
    case Status::Unsupported: return GS130_UNSUPPORTED;
    case Status::NotFound:    return GS130_NOT_FOUND;
    case Status::HwError:     return GS130_HW_ERROR;
    case Status::Timeout:     return GS130_TIMEOUT;
    case Status::ThreadClosed:return GS130_THREAD_CLOSED;
    }
    return GS130_HW_ERROR;
}

/* Internal device state */
struct gs130_device_s {
    std::mutex              mtx;   // API lock: every public function takes it; threads never do

    std::unique_ptr<eeprom::Eeprom>     eeprom;
    std::unique_ptr<imu::Imu>           imu;
    std::unique_ptr<pipeline::Pipeline> pipeline;

    // Data queues: background threads enqueue frames/packets; the user dequeues them
    std::unique_ptr<
        base::Fifo<
            std::array<
                gs130_image_nv12_t, static_cast<std::size_t>(CamIndex::Num)
            >>> camera_fifo;   // [CamIndex::Right]=0, [CamIndex::Left]=1
    std::unique_ptr<base::Fifo<gs130_imu_packet_t>> imu_fifo;

    StereoImuModel       cal_internal;

    // Threads
    std::thread              camera_thread;
    std::thread              imu_thread;
    std::atomic<bool>        camera_on{false}; // set once IMU finds FSYNC; the camera waits for it to start streaming
    std::atomic<Status>      state{Status::ThreadClosed}; // Ok = running, ThreadClosed = idle, anything else = faulted (deinit() clears)

    // Timestamp tracker (master = camera LPWM frame period, slave = IMU FSYNC)
    std::unique_ptr<base::TimestampTracker> tracker;

    // Cached from the config at init (used by the threads)
    uint32_t output_w = 0, output_h = 0;
    gs130_stereo_layout_t stereo_layout = GS130_STEREO_LAYOUT_NONE;
    uint32_t fps = 0;
    uint16_t accel_fsr_g = 0, gyro_fsr_dps = 0;
    CamIndex fsync_camera = CamIndex::Right;   // eye bound to IMU FSYNC
};

/* ---- Calibration helpers (3x3 row-major) ---- */

static void mat33_mul(const double *A, const double *B, double *C)   // C = A * B
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[i*3+0]*B[0*3+j] + A[i*3+1]*B[1*3+j] + A[i*3+2]*B[2*3+j];
}

static void mat33_t_mul(const double *A, const double *B, double *C)   // C = A^T * B
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[0*3+i]*B[0*3+j] + A[1*3+i]*B[1*3+j] + A[2*3+i]*B[2*3+j];
}

static void mat33_mul_t(const double *A, const double *B, double *C)   // C = A * B^T
{
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            C[i*3+j] = A[i*3+0]*B[j*3+0] + A[i*3+1]*B[j*3+1] + A[i*3+2]*B[j*3+2];
}

static void mat33_mul_vec(const double *A, const double *v, double *out)   // out = A * v
{
    for(int i = 0; i < 3; i++)
        out[i] = A[i*3+0]*v[0] + A[i*3+1]*v[1] + A[i*3+2]*v[2];
}

// stored extrinsics (sensor -> reference frame) of a frame; {nullptr, nullptr} on invalid frame
static std::pair<double*, double*> frame_extrinsics(gs130_device_t *dev, gs130_reference_frame_t frame)
{
    switch(frame){
        case GS130_REF_CAMERA_RIGHT: return {dev->cal_internal.cam_right_R, dev->cal_internal.cam_right_T};
        case GS130_REF_CAMERA_LEFT:  return {dev->cal_internal.cam_left_R,  dev->cal_internal.cam_left_T};
        case GS130_REF_IMU:          return {dev->cal_internal.imu_R,       dev->cal_internal.imu_T};
        default:                     return {nullptr, nullptr};
    }
}

/* ---- Background threads ---- */

// IMU packet cache limit: exceeding it means the FSYNC signal is lost
constexpr std::size_t kMaxImuCache = 1000;

// IMU thread: drain the hardware FIFO, detect the FSYNC handshake, feed the
// timestamp tracker, and pair cached packets with corrected timestamps
static void imu_thread_func(gs130_device_t *dev)
{
    const double accel_scale = (double)dev->accel_fsr_g * 9.80665 / 32768.0;
    const double gyro_scale  = (double)dev->gyro_fsr_dps * 3.14159265358979323846 / 180.0 / 32768.0;

    ImuHwFifo16Packet raw[128]; // hardware FIFO holds up to 128 packets
    std::vector<ImuHwFifo16Packet> cache;
    std::vector<gs130_imu_packet_t> pending; // staging area, enqueued oldest to newest

    bool available = false;

    while(dev->state.load() == Status::Ok) {
        if(dev->imu->full()){dev->state.store(Status::HwError);return ;}

        std::size_t n = 0;
        Status st = dev->imu->read(raw, 128, &n);
        if(st != Status::Ok || n == 0) { usleep(1000); continue; }

        // Iterate packets
        for(std::size_t i = 0; i < n; i++){
            // cache too large = no FSYNC anchor for a long time, latch a fault
            if(cache.size() > kMaxImuCache){dev->state.store(Status::HwError); return ;}

            // Try the startup handshake
            if(!available){
                // the latest packet being an FSYNC packet means sync is established
                if(i+1==n && raw[i].is_fsync){
                    dev->camera_on.store(true);
                    available = true;
                }
            }
            // Post-handshake work
            else{
                // feed the tracker (FSYNC packets carry the edge offset, normal packets do not)
                const uint64_t delta_ns = static_cast<uint64_t>(raw[i].delta_time_us) * 1000;
                dev->tracker->feed_slave_sample(raw[i].is_fsync ? &delta_ns : nullptr);
                // cache the packet
                cache.push_back(raw[i]);

                // pair only after an FSYNC anchor (only FSYNC generates timestamps into ready_)
                if(raw[i].is_fsync){
                    // take all timestamps (newest to oldest), pair with the cache tail (newest to oldest), match as many as possible
                    const std::vector<uint64_t> ts_list = dev->tracker->take_ready();
                    std::size_t k = 0;
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
                    // whichever side runs out first, clear both remainders
                    cache.clear();
                    // publish: push into imu_fifo oldest to newest
                    for(auto it = pending.rbegin(); it != pending.rend(); ++it)
                        dev->imu_fifo->push(*it);
                    pending.clear();
                }
            }
        }
        if(dev->state.load() != Status::Ok)break;
    }
}

// Camera thread: wait for the FSYNC handshake, start the stream, then fetch
// half-frame-aligned stereo pairs and push them into the FIFO
static void camera_thread_func(gs130_device_t *dev)
{
    while(dev->state.load() == Status::Ok && !dev->camera_on.load())usleep(1000);
    if(dev->state.load() != Status::Ok)return ;

    const Status st_start = dev->pipeline->start(dev->fsync_camera);
    if(st_start != Status::Ok){dev->state.store(st_start);return ;}

    const uint64_t cycle_ns = 1000000000ULL / dev->fps / 2;
    const size_t R = static_cast<std::size_t>(CamIndex::Right);
    const size_t L = static_cast<std::size_t>(CamIndex::Left);

    while(dev->state.load() == Status::Ok){
        std::array<gs130_image_nv12_t, static_cast<std::size_t>(CamIndex::Num)> frame{};
        Status st;
        uint64_t ts[static_cast<std::size_t>(CamIndex::Num)] = {0, 0};
        uint8_t *y[static_cast<std::size_t>(CamIndex::Num)]  = {nullptr, nullptr};
        uint8_t *uv[static_cast<std::size_t>(CamIndex::Num)] = {nullptr, nullptr};
        uint32_t stride;
        const uint32_t width  = dev->output_w;
        const uint32_t height = dev->output_h;

        // fetch the FSYNC-bound eye (master clock) first, then the other eye
        const CamIndex F = dev->fsync_camera;
        const CamIndex O = (F == CamIndex::Right) ? CamIndex::Left : CamIndex::Right;
        const size_t fi = static_cast<std::size_t>(F);
        const size_t oi = static_cast<std::size_t>(O);

        // allocate and lay out the frame buffer(s) for this capture
        switch(dev->stereo_layout){
            case GS130_STEREO_LAYOUT_NONE:
                stride = width;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3 / 2);
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3 / 2);
                frame[R].data = y[R];
                frame[L].data = y[L];
                if(!y[R] || !y[L])goto free_frame;
                uv[R] = y[R] + (std::size_t)width * height;
                uv[L] = y[L] + (std::size_t)width * height;
                frame[R].width = width; frame[R].height = height;
                frame[L].width = width; frame[L].height = height;
                break;
            case GS130_STEREO_LAYOUT_LEFT_RIGHT:
                stride = width * 2;
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[L];
                if(!y[L])goto free_frame;
                y[R]  = y[L] + width;
                uv[L] = y[L] + (std::size_t)width * height * 2;
                uv[R] = uv[L] + width;
                frame[0].width = width * 2; frame[0].height = height;
                break;
            case GS130_STEREO_LAYOUT_RIGHT_LEFT:
                stride = width * 2;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[R];
                if(!y[R])goto free_frame;
                y[L]  = y[R] + width;
                uv[R] = y[R] + (std::size_t)width * height * 2;
                uv[L] = uv[R] + width;
                frame[0].width = width * 2; frame[0].height = height;
                break;
            case GS130_STEREO_LAYOUT_TOP_BOTTOM:
                stride = width;
                y[L] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[L];
                if(!y[L])goto free_frame;
                y[R]  = y[L] + (std::size_t)width * height;
                uv[L] = y[L] + (std::size_t)width * height * 2;
                uv[R] = uv[L] + (std::size_t)width * height / 2;
                frame[0].width = width; frame[0].height = height * 2;
                break;
            case GS130_STEREO_LAYOUT_BOTTOM_TOP:
                stride = width;
                y[R] = (uint8_t *)malloc((std::size_t)width * height * 3);
                frame[0].data = y[R];
                if(!y[R])goto free_frame;
                y[L]  = y[R] + (std::size_t)width * height;
                uv[R] = y[R] + (std::size_t)width * height * 2;
                uv[L] = uv[R] + (std::size_t)width * height / 2;
                frame[0].width = width; frame[0].height = height * 2;
                break;
            default:
                goto free_frame;   // invalid layout (validated at init, unreachable)
        }

        st = dev->pipeline->get_frame(F, y[fi], uv[fi],
                                      width, height, stride, stride, &ts[fi], 100);
        if(st != Status::Ok)goto free_frame;
        // upload the master clock phase from the bound eye's new frame first
        dev->tracker->update_master_timestamp_ns(ts[fi]);

        st = dev->pipeline->get_frame(O, y[oi], uv[oi],
                                      width, height, stride, stride, &ts[oi], 100);
        if(st != Status::Ok)goto free_frame;

        // half-frame alignment: drop the eye with the smaller (older) timestamp and refetch until aligned
        for(;;) {
            uint64_t d = (ts[R] > ts[L]) ? (ts[R] - ts[L]) : (ts[L] - ts[R]);
            if(d <= cycle_ns)break;
            if(dev->state.load() != Status::Ok)goto free_frame;

            const size_t lag = (ts[R] < ts[L]) ? R : L;   // smaller timestamp = older frame
            const CamIndex lag_idx = (lag == R) ? CamIndex::Right : CamIndex::Left;
            st = dev->pipeline->get_frame(lag_idx, y[lag], uv[lag],
                                          width, height, stride, stride, &ts[lag], 100);
            if(st != Status::Ok)goto free_frame;
            if(lag == fi)   // refetched the bound eye; upload the phase again
                dev->tracker->update_master_timestamp_ns(ts[fi]);
        }

        if(dev->stereo_layout == GS130_STEREO_LAYOUT_NONE){
            frame[R].timestamp_ns = ts[R];
            frame[L].timestamp_ns = ts[L];
        } else {
            frame[0].timestamp_ns = ts[fi];   // combined frame keeps the FSYNC-bound eye's timestamp
        }

        if(!dev->camera_fifo->push(frame))goto free_frame;
        continue;

free_frame:
        for(std::size_t i = 0; i < static_cast<std::size_t>(CamIndex::Num); i++) { free(frame[i].data); }
    }
}

/* ---- Public C API (skeleton) ---- */

extern "C" {

#ifndef GS130_VERSION
#  define GS130_VERSION "unknown"
#endif

const char *gs130_version(void)
{
    return GS130_VERSION;
}

#ifndef GS130_PLATFORM
#  define GS130_PLATFORM "unknown"
#endif

const char *gs130_platform(void)
{
    return GS130_PLATFORM;
}

gs130_device_t *gs130_create()
{
    return new gs130_device_t;   // state starts at GS130_THREAD_CLOSED
}

void gs130_destroy(
    gs130_device_t *dev)
{
    delete dev;   // call gs130_deinit() first: destroying with running threads is undefined behavior
}

gs130_err_t gs130_init(
    gs130_device_t *dev,
    const gs130_config_t *cfg)
{
    if(dev == nullptr || cfg == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->eeprom || dev->pipeline || dev->imu)return GS130_PARAM_ERROR;   // already initialized
    if(cfg->camera_config.stereo_layout > GS130_STEREO_LAYOUT_BOTTOM_TOP)return GS130_PARAM_ERROR;

    // Create the EEPROM object: probe candidate buses in order, leave null on failure
    for(std::size_t i = 0; i < cfg->eeprom_config.bus_num; i++){
        dev->eeprom.reset(
            new eeprom::Eeprom(cfg->eeprom_config.bus[i], cfg->eeprom_config.addr));
        if(*dev->eeprom)break;
    }
    if(dev->eeprom && !*dev->eeprom)dev->eeprom.reset();

    // Create the IMU object: probe candidate buses in order, leave null on failure
    for(std::size_t i = 0; i < cfg->imu_config.bus_num; i++) {
        dev->imu.reset(
            new imu::Imu(cfg->imu_config.bus[i], cfg->imu_config.addr));
        if(*dev->imu)break;
    }
    if(dev->imu && !*dev->imu)dev->imu.reset();

    // Create the Pipeline object
    dev->pipeline.reset(
        new pipeline::Pipeline(
            cfg->camera_config.left_addr, cfg->camera_config.right_addr,
            cfg->camera_config.bus, cfg->camera_config.bus_num));
    if(!*dev->pipeline)return GS130_NOT_FOUND;

    // Create the FIFOs (the Fifo constructor leaves itself invalid when depth < 2)
    dev->camera_fifo.reset(new base::Fifo<std::array<gs130_image_nv12_t, 2>>(
        cfg->camera_fifo.depth, static_cast<FifoMode>(cfg->camera_fifo.mode)));
    dev->imu_fifo.reset(new base::Fifo<gs130_imu_packet_t>(
        cfg->imu_fifo.depth, static_cast<FifoMode>(cfg->imu_fifo.mode)));

    // Read the EEPROM calibration (read if probed; map failures to error codes)
    if(dev->eeprom) {
        Status st = dev->eeprom->read(&dev->cal_internal);
        if(st != Status::Ok)return to_err(st);
    }

    // Initialize the IMU (configure only, no streaming; if probed, configuration must succeed)
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

    // Cache the parameters needed by the threads
    dev->output_w = cfg->camera_config.output_width;
    dev->output_h = cfg->camera_config.output_height;
    dev->stereo_layout = cfg->camera_config.stereo_layout;
    dev->fps      = cfg->camera_config.fps;
    dev->accel_fsr_g  = cfg->imu_config.accel_fsr_g;
    dev->gyro_fsr_dps = cfg->imu_config.gyro_fsr_dps;
    dev->fsync_camera = (cfg->camera_config.fsync_camera == GS130_CAMERA_RIGHT_IDX)
                             ? CamIndex::Right : CamIndex::Left;

    // Timestamp tracker (master = camera LPWM frame period, slave = IMU FSYNC)
    dev->tracker.reset(new base::TimestampTracker(1000000000ULL / dev->fps));

    // Initialize the camera (stream setup config, no streaming)
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

gs130_err_t gs130_deinit(
    gs130_device_t *dev)
{
    if(dev == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);

    gs130_stop(dev);   // joins the threads if they are running

    dev->state.store(Status::ThreadClosed);   // clear any latched fault so the device can be re-initialized
    dev->pipeline.reset();   // triggers destructor: tear down stream + restore sensor power-on state
    dev->imu.reset();        // close I2C
    dev->eeprom.reset();     // close I2C
    dev->camera_fifo.reset();
    dev->imu_fifo.reset();
    return GS130_OK;
}

gs130_err_t gs130_start(
    gs130_device_t *dev)
{
    if(dev == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);

    const Status st = dev->state.load();
    if(st == Status::Ok)return GS130_PARAM_ERROR;            // already started
    if(st != Status::ThreadClosed)return to_err(st);               // faulted: deinit() first
    if(!dev->pipeline)return GS130_PARAM_ERROR;           // not initialized

    dev->camera_on = false;
    dev->state.store(Status::Ok);

    if(dev->imu) {
        Status s = dev->imu->start();
        if(s != Status::Ok) { dev->state.store(Status::ThreadClosed); return to_err(s); }
        dev->imu_thread = std::thread(imu_thread_func, dev);
    } else {
        dev->camera_on = true;   // no IMU: skip the handshake, start streaming directly
    }
    dev->camera_thread = std::thread(camera_thread_func, dev);

    return GS130_OK;
}

void gs130_stop(
    gs130_device_t *dev)
{
    if(dev == nullptr)return;
    std::lock_guard<std::mutex> lock(dev->mtx);

    // tell the threads to exit; a latched fault (if any) is preserved
    Status expected = Status::Ok;
    dev->state.compare_exchange_strong(expected, Status::ThreadClosed);

    if(!dev->camera_thread.joinable() && !dev->imu_thread.joinable())return;   // never started
    if(dev->camera_thread.joinable())dev->camera_thread.join();
    if(dev->imu_thread.joinable())dev->imu_thread.join();
    dev->pipeline->stop();
    if(dev->imu)dev->imu->stop();
}

/* ---- Camera data ---- */

size_t gs130_available_camera(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->camera_fifo)return 0;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return 0;
    return dev->camera_fifo->size();
}

gs130_err_t gs130_get_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image_left,
    gs130_image_nv12_t *image_right)
{
    if(dev == nullptr || image_left == nullptr || image_right == nullptr)
        return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return GS130_THREAD_CLOSED;
    if(dev->stereo_layout != GS130_STEREO_LAYOUT_NONE)return GS130_UNSUPPORTED;
    if(!dev->camera_fifo)return GS130_PARAM_ERROR;

    std::array<gs130_image_nv12_t, 2> f;
    if(!dev->camera_fifo->pop(f))return GS130_TIMEOUT;

    *image_left  = f[static_cast<std::size_t>(CamIndex::Left)];
    *image_right = f[static_cast<std::size_t>(CamIndex::Right)];
    return GS130_OK;
}

gs130_err_t gs130_get_stereo_nv12_frame(
    gs130_device_t *dev,
    gs130_image_nv12_t *image)
{
    if(dev == nullptr || image == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return GS130_THREAD_CLOSED;
    if(dev->stereo_layout == GS130_STEREO_LAYOUT_NONE)return GS130_UNSUPPORTED;
    if(!dev->camera_fifo)return GS130_PARAM_ERROR;

    std::array<gs130_image_nv12_t, 2> f;
    if(!dev->camera_fifo->pop(f))return GS130_TIMEOUT;

    *image = f[0];   // zero-copy: the combined frame was spliced by the camera thread
    return GS130_OK;
}

/* ---- IMU data ---- */

size_t gs130_available_imu(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->imu || !dev->imu_fifo)return 0;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return 0;
    return dev->imu_fifo->size();
}

gs130_err_t gs130_get_imu_packet(
    gs130_device_t *dev,
    gs130_imu_packet_t *out)
{
    if(dev == nullptr || out == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(dev->state.load() != Status::Ok)return GS130_THREAD_CLOSED;
    if(!dev->imu || !dev->imu_fifo)return GS130_PARAM_ERROR;
    if(!dev->imu_fifo->pop(*out))return GS130_TIMEOUT;
    return GS130_OK;
}

const char *gs130_get_imu_name(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->imu)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->imu->name();
}

const char *gs130_get_imu_info(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->imu)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->imu->info();
}

/* ---- Calibration ---- */

gs130_err_t gs130_get_camera_intrinsics(
    gs130_device_t *dev,
    gs130_camera_index_t cam_idx,
    gs130_camera_intrinsics_t *intrinsics)
{
    if(dev == nullptr || intrinsics == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    if(cam_idx != GS130_CAMERA_LEFT_IDX && cam_idx != GS130_CAMERA_RIGHT_IDX)return GS130_PARAM_ERROR;
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    const CameraIntrinsics &src = (cam_idx == GS130_CAMERA_LEFT_IDX) ? dev->cal_internal.cam_left
                                                                     : dev->cal_internal.cam_right;
    intrinsics->fx = src.fx; intrinsics->fy = src.fy;
    intrinsics->cx = src.cx; intrinsics->cy = src.cy;
    memcpy(intrinsics->K, src.K, sizeof(src.K));
    memcpy(intrinsics->dist_coeffs, src.dist_coeffs, sizeof(src.dist_coeffs));
    intrinsics->dist_model = (src.dist_model == DistModel::Fisheye) ? GS130_DIST_FISHEYE
                                                                    : GS130_DIST_PINHOLE;
    return GS130_OK;
}

gs130_err_t gs130_get_imu_intrinsics(
    gs130_device_t *dev,
    gs130_imu_intrinsics_t *intrinsics)
{
    if(dev == nullptr || intrinsics == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    const ImuIntrinsics &src = dev->cal_internal.imu;
    memcpy(intrinsics->accel_misalign, src.accel_misalign, sizeof(src.accel_misalign));
    memcpy(intrinsics->accel_scale,    src.accel_scale,    sizeof(src.accel_scale));
    memcpy(intrinsics->accel_bias,     src.accel_bias,     sizeof(src.accel_bias));
    intrinsics->accel_noise       = src.accel_noise;
    intrinsics->accel_random_walk = src.accel_random_walk;
    memcpy(intrinsics->gyro_misalign, src.gyro_misalign, sizeof(src.gyro_misalign));
    memcpy(intrinsics->gyro_scale,    src.gyro_scale,    sizeof(src.gyro_scale));
    memcpy(intrinsics->gyro_bias,     src.gyro_bias,     sizeof(src.gyro_bias));
    intrinsics->gyro_noise       = src.gyro_noise;
    intrinsics->gyro_random_walk = src.gyro_random_walk;
    return GS130_OK;
}

gs130_err_t gs130_get_relative_R(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *R)
{
    if(dev == nullptr || R == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    const auto from = frame_extrinsics(dev, from_frame);
    const auto to   = frame_extrinsics(dev, to_frame);
    if(from.first == nullptr || to.first == nullptr)return GS130_PARAM_ERROR;

    mat33_t_mul(to.first, from.first, R);   // R = R_to^T * R_from
    return GS130_OK;
}

gs130_err_t gs130_get_relative_T(
    gs130_device_t *dev,
    gs130_reference_frame_t from_frame,
    gs130_reference_frame_t to_frame,
    double *T)
{
    if(dev == nullptr || T == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    const auto from = frame_extrinsics(dev, from_frame);
    const auto to   = frame_extrinsics(dev, to_frame);
    if(from.first == nullptr || to.first == nullptr)return GS130_PARAM_ERROR;

    // T = R_to^T * (T_from - T_to)
    const double d[3] = {from.second[0] - to.second[0],
                         from.second[1] - to.second[1],
                         from.second[2] - to.second[2]};
    mat33_t_mul(to.first, d, T);
    return GS130_OK;
}

gs130_err_t gs130_get_calibration(
    gs130_device_t *dev,
    gs130_calibration_t *calibration)
{
    if(dev == nullptr || calibration == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    static_assert(sizeof(gs130_calibration_t) == sizeof(StereoImuModel),
                  "C calibration struct and internal StereoImuModel must be layout-compatible");
    memcpy(calibration, &dev->cal_internal, sizeof(*calibration));
    return GS130_OK;
}

gs130_err_t gs130_convert_calibration(
    gs130_device_t *dev,
    gs130_reference_frame_t ref_frame,
    const double ref_R[9],
    const double ref_T[3])
{
    if(dev == nullptr || ref_R == nullptr || ref_T == nullptr)return GS130_PARAM_ERROR;
    std::lock_guard<std::mutex> lock(dev->mtx);
    const Status st = dev->state.load();
    if(st != Status::Ok && st != Status::ThreadClosed)return GS130_THREAD_CLOSED;   // faulted
    if(!dev->eeprom)return GS130_NOT_FOUND;

    const auto anchor = frame_extrinsics(dev, ref_frame);
    if(anchor.first == nullptr)return GS130_PARAM_ERROR;

    // Re-anchor: G maps old reference frame -> new one; afterwards the chosen frame's
    // extrinsic becomes exactly (ref_R, ref_T), all others keep their relative poses:
    //   G_R = ref_R * R_anchor^T,  G_T = ref_T - G_R * T_anchor
    //   R_x' = G_R * R_x,          T_x' = G_R * T_x + G_T
    double GR[9], GT[3], tmp[3];
    mat33_mul_t(ref_R, anchor.first, GR);
    mat33_mul_vec(GR, anchor.second, tmp);
    for(int i = 0; i < 3; i++)GT[i] = ref_T[i] - tmp[i];

    double *all_R[3] = {dev->cal_internal.cam_right_R, dev->cal_internal.cam_left_R, dev->cal_internal.imu_R};
    double *all_T[3] = {dev->cal_internal.cam_right_T, dev->cal_internal.cam_left_T, dev->cal_internal.imu_T};
    for(int k = 0; k < 3; k++){
        double R2[9], T2[3];
        mat33_mul(GR, all_R[k], R2);
        mat33_mul_vec(GR, all_T[k], T2);
        for(int i = 0; i < 3; i++)T2[i] += GT[i];
        memcpy(all_R[k], R2, sizeof(R2));
        memcpy(all_T[k], T2, sizeof(T2));
    }
    return GS130_OK;
}

const char *gs130_get_eeprom_name(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->eeprom)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->eeprom->name();
}

const char *gs130_get_eeprom_info(
    gs130_device_t *dev)
{
    if(dev == nullptr || !dev->eeprom)return nullptr;
    std::lock_guard<std::mutex> lock(dev->mtx);
    return dev->eeprom->info();
}

} // extern "C"
