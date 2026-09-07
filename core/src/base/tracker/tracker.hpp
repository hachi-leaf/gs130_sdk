/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * Master-Slave Timestamp Tracker, Thread-safe
 */
#ifndef GS130_BASE_TRACKER_HPP
#define GS130_BASE_TRACKER_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace gs130 {
namespace base {

class TimestampTracker {
public:
    explicit TimestampTracker(uint32_t master_cycle_ns);

    // copying disabled
    TimestampTracker(const TimestampTracker &) = delete;             
    TimestampTracker &operator=(const TimestampTracker &) = delete;

    // moving allowed (std::mutex is not movable, so the lock is held via unique_ptr; the source object is unusable after the move)
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

    // upload the master clock's absolute time as the phase reference
    void update_master_timestamp_ns(uint64_t timestamp_ns);

    // upload a slave clock sample. delta == NULL means an ordinary sample;
    // otherwise it is an anchor sample (offset from trigger edge to sample point, ns).
    void feed_slave_sample(const uint64_t *delta_time_ns);

    // pop corrected timestamps of ready samples, newest first; returns false when none remain.
    // the first call returns the current frame (anchor sample), then the previous frame, the one before, ...
    bool get_timestamp(uint64_t *slave_timestamp_ns);

    // number of ready timestamps (for assertion before external pairing)
    size_t ready_count() const;

    // take all ready timestamps (newest first: first element = current anchor sample) and clear them
    std::vector<uint64_t> take_ready();

    // clear ready timestamps (discarding unpaired ones)
    void clear_ready();

private:
    // wrap-align to the master clock phase; the caller must hold mtx_ through a public method
    uint64_t align_master_phase(uint64_t predicted) const;

    // absolute time provided by the master clock side
    struct MasterClock {
        uint64_t first_ns = 0;   // first frame
        uint64_t last_ns = 0;    // latest (phase reference)
    };

    enum class Phase { WaitFirstAnchor, WaitTwoMaster, WaitKeyPoint, Tracking };

    struct SlaveCounter {
        Phase phase = Phase::WaitFirstAnchor;
        uint64_t last_anchor_edge_timestamp_ns = 0;   // master timestamp of the previous anchor point
        uint64_t last_anchor_sample_timestamp_ns = 0; // slave timestamp of the previous anchor

        // WaitFirstAnchor
        bool has_first_anchor = false;
        uint64_t first_delta_time_ns = 0;

        uint32_t anchor_count = 0;
        uint32_t sample_count = 0;
    };

    uint32_t master_cycle_ns_;    // master clock nominal period (frame period)

    MasterClock master_;
    SlaveCounter slave_;

    std::vector<uint64_t> ready_;    // ready timestamps (get pops from the tail)

    std::unique_ptr<std::mutex> mtx_;
};

} // namespace base
} // namespace gs130

#endif // GS130_BASE_TRACKER_HPP