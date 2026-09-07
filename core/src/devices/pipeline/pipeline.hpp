/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef GS130_DEVICES_PIPELINE_PIPELINE_HPP
#define GS130_DEVICES_PIPELINE_PIPELINE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "types.hpp"

namespace gs130 {
namespace pipeline {

// Stereo pipeline. The platform implementation is selected at compile time via a -D
// macro (platform SDK headers cannot coexist); platform-private state hides inside
// Impl, so C headers such as HBN never leak into this header.
class Pipeline {
public:
    Pipeline(uint8_t left_addr, uint8_t right_addr,
             const uint8_t *bus_list, size_t bus_num);
    ~Pipeline();   // calls deinit to release hardware

    Pipeline(const Pipeline &)            = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    // both sensors probed successfully
    explicit operator bool() const;

    Status init(const PipelineConfig &cfg, StereoImuModel *cal);
    void   deinit();   // release hardware acquired by init (also called from the destructor as a fallback)
    Status start(CamIndex first);   // start the 'first' channel, then the other
    void   stop();

    // fetch one frame into user buffers: copy row by row using y_stride / uv_stride; width/height mismatch -> ParamError
    Status get_frame(CamIndex idx,
                     uint8_t *y, uint8_t *uv,
                     uint32_t width, uint32_t height,
                     uint32_t y_stride, uint32_t uv_stride,
                     uint64_t *timestamp_ns,
                     uint32_t timeout_ms);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace pipeline
} // namespace gs130

#endif // GS130_DEVICES_PIPELINE_PIPELINE_HPP
