/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * Master-Slave Timestamp Tracker, Thread-safe
 */
#ifndef GS130W_BASE_TRACKER_HPP
#define GS130W_BASE_TRACKER_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace gs130w {
namespace base {

class TimestampTracker {
public:
    explicit TimestampTracker(uint32_t master_cycle_ns);

    // 禁止拷贝
    TimestampTracker(const TimestampTracker &) = delete;             
    TimestampTracker &operator=(const TimestampTracker &) = delete;

    // 允许移动（std::mutex 不可移动，锁存放在 unique_ptr 中；移动后源对象不可再用）
    TimestampTracker(TimestampTracker &&other) noexcept
        : master_cycle_ns_(other.master_cycle_ns_), master_(other.master_),
          slave_(other.slave_), ready_(std::move(other.ready_)),
          mtx_(std::move(other.mtx_))
    {
    }
    TimestampTracker &operator=(TimestampTracker &&other) noexcept
    {
        if(this != &other){
            master_cycle_ns_ = other.master_cycle_ns_;
            master_          = other.master_;
            slave_           = other.slave_;
            ready_           = std::move(other.ready_);
            mtx_             = std::move(other.mtx_);
        }
        return *this;
    }

    // 上传主时钟绝对时间作为相位参考
    void update_master_timestamp_ns(uint64_t timestamp_ns);

    // 上传从时钟样本。delta 为 NULL 表示普通样本；
    // 否则为锚点样本（触发沿到采样点偏移，ns）。
    void feed_slave_sample(const uint64_t *delta_time_ns);

    // 从新到旧弹出就绪样本的修正时间戳；无更多返回 false。
    // 首次返回本帧（锚点样本），之后依次是前一帧、再前一帧……
    bool get_timestamp(uint64_t *slave_timestamp_ns);

    // 就绪时间戳数量（外侧配对前断言用）
    size_t ready_count() const;

    // 取出全部就绪时间戳（新到旧：首元素=当前锚点样本），同时清空
    std::vector<uint64_t> take_ready();

    // 清空就绪时间戳（丢弃未配对的）
    void clear_ready();

private:
    // 回卷对齐主时钟相位；调用前须由公开方法持有 mtx_
    uint64_t align_master_phase(uint64_t predicted) const;

    // 主时钟侧给的绝对时间
    struct MasterClock {
        uint64_t first_ns = 0;   // 首帧
        uint64_t last_ns = 0;    // 最新（相位参考）
    };

    enum class Phase { WaitFirstAnchor, WaitTwoMaster, WaitKeyPoint, Tracking };

    struct SlaveCounter {
        Phase phase = Phase::WaitFirstAnchor;
        uint64_t last_anchor_edge_timestamp_ns = 0;   // 上个锚定点主时间戳
        uint64_t last_anchor_sample_timestamp_ns = 0; // 上个锚定从时间戳

        // WaitFirstAnchor
        bool has_first_anchor = false;
        uint64_t first_delta_time_ns = 0;

        uint32_t anchor_count = 0;
        uint32_t sample_count = 0;
    };

    uint32_t master_cycle_ns_;    // 主时钟名义周期（帧周期）

    MasterClock master_;
    SlaveCounter slave_;

    std::vector<uint64_t> ready_;    // 就绪时间戳（get 从尾部弹出）

    std::unique_ptr<std::mutex> mtx_;
};

} // namespace base
} // namespace gs130w

#endif // GS130W_BASE_TRACKER_HPP