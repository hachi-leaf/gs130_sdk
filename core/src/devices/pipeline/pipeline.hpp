/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#ifndef GS130W_DEVICES_PIPELINE_PIPELINE_HPP
#define GS130W_DEVICES_PIPELINE_PIPELINE_HPP

#include <cstddef>
#include <cstdint>
#include <memory>

#include "types.hpp"

namespace gs130w {
namespace pipeline {

// 双目 pipeline。平台实现在编译期由 -D 宏选定（各平台 SDK 头文件互不共存），
// 平台私有状态藏在 Impl 内，HBN 等 C 头文件不会泄漏到本头文件。
class Pipeline {
public:
    Pipeline(uint8_t left_addr, uint8_t right_addr,
             const uint8_t *bus_list, size_t bus_num);
    ~Pipeline();   // 调 deinit 释放硬件

    Pipeline(const Pipeline &)            = delete;
    Pipeline &operator=(const Pipeline &) = delete;

    // 两路 sensor 探测成功
    explicit operator bool() const;

    Status init(const PipelineConfig &cfg, StereoImuModel *cal);
    void   deinit();   // 释放 init 占用的硬件（析构兜底调用）
    Status start(CamIndex first);   // 先开 first 那一路，再开另一路
    void   stop();

    // 取一帧到用户缓冲：逐行按 y_stride / uv_stride 拷贝；宽高不符报 ParamError
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
} // namespace gs130w

#endif // GS130W_DEVICES_PIPELINE_PIPELINE_HPP
