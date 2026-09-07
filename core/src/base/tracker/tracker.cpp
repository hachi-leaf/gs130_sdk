/**
 * @file tracker.hpp
 * @brief Master-slave timestamp tracker; thread-safe.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "base/tracker/tracker.hpp"
#include <cstdio>

namespace gs130 {
namespace base {

TimestampTracker::TimestampTracker(uint32_t master_cycle_ns)
    : master_cycle_ns_(master_cycle_ns), mtx_(std::make_unique<std::mutex>())
{}

void TimestampTracker::update_master_timestamp_ns(
    uint64_t timestamp_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    if(master_.first_ns == 0)master_.first_ns = timestamp_ns;
    master_.last_ns = timestamp_ns;

}

// Lock-free; the caller (a public method) holds the lock
uint64_t TimestampTracker::align_master_phase(
    uint64_t predicted) const
{
    uint64_t aligned = master_.last_ns;

    while(aligned > predicted + master_cycle_ns_ / 2)aligned -= master_cycle_ns_;
    while(aligned < predicted - master_cycle_ns_ / 2)aligned += master_cycle_ns_;

    return aligned;
}

void TimestampTracker::feed_slave_sample(
    const uint64_t *delta_time_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    const bool fsync = (delta_time_ns != nullptr);

    switch(slave_.phase){
    case Phase::WaitFirstAnchor: // wait for the first anchor
        // first-anchor branch
        if(fsync && !slave_.has_first_anchor){
            slave_.has_first_anchor = true;
            slave_.first_delta_time_ns = *delta_time_ns;
            slave_.phase = Phase::WaitTwoMaster;
        }
        return ;

    case Phase::WaitTwoMaster:
        // wait for two Master clocks to arrive
        if(master_.first_ns != 0 && master_.first_ns < master_.last_ns)
            slave_.phase = Phase::WaitKeyPoint;

        [[fallthrough]];

    case Phase::WaitKeyPoint: // key frame after two Masters
        // both Phase::WaitTwoMaster and Phase::WaitKeyPoint conditions are met
        if(fsync && slave_.phase == Phase::WaitKeyPoint){
            slave_.phase = Phase::Tracking;
            slave_.last_anchor_edge_timestamp_ns = master_.first_ns;
            slave_.last_anchor_sample_timestamp_ns = master_.first_ns + slave_.first_delta_time_ns;
        }

        // count during the wait phases
        if(fsync)slave_.anchor_count++;
        slave_.sample_count++;

        // intercept non-anchor frames
        if(slave_.phase != Phase::Tracking)return;
        
        // the anchor frame happens to share the Tracking-phase computation
        [[fallthrough]];

    case Phase::Tracking:
        // anchor-frame computation
        if(fsync){
            uint64_t anchor_edge_timestamp_ns = 
                slave_.last_anchor_edge_timestamp_ns + slave_.anchor_count * master_cycle_ns_;
            anchor_edge_timestamp_ns = align_master_phase(anchor_edge_timestamp_ns);
            uint64_t anchor_sample_timestamp_ns = anchor_edge_timestamp_ns + *delta_time_ns;

            // compute the slave-clock interval
            uint64_t slave_gap_ns = 
                (anchor_sample_timestamp_ns - slave_.last_anchor_sample_timestamp_ns) / slave_.sample_count;

            // push into ready_: ordinary samples step by gap; the last one is the current anchor
            for(uint32_t j = 1; j <= slave_.sample_count; j++)
                ready_.push_back(slave_.last_anchor_sample_timestamp_ns + slave_gap_ns * j);

            slave_.last_anchor_edge_timestamp_ns = anchor_edge_timestamp_ns;
            slave_.last_anchor_sample_timestamp_ns = anchor_sample_timestamp_ns;

            slave_.sample_count = 0;
        }

        // count the current sample (the one with FSYNC) into the new period
        slave_.sample_count++;
        slave_.anchor_count = 1;
        return ;
    }
}

bool TimestampTracker::get_timestamp(uint64_t *slave_timestamp_ns)
{
    std::lock_guard<std::mutex> lock(*mtx_);
    if(!slave_timestamp_ns || ready_.empty())return false;
    *slave_timestamp_ns = ready_.back();
    ready_.pop_back();
    return true;
}

std::size_t TimestampTracker::ready_count() const
{
    std::lock_guard<std::mutex> lock(*mtx_);
    return ready_.size();
}

std::vector<uint64_t> TimestampTracker::take_ready()
{
    std::lock_guard<std::mutex> lock(*mtx_);
    std::vector<uint64_t> out;
    out.reserve(ready_.size());
    // newest to oldest: back -> front; first element = current anchor sample
    for(auto it = ready_.rbegin(); it != ready_.rend(); ++it)
        out.push_back(*it);
    ready_.clear();
    return out;
}

void TimestampTracker::clear_ready()
{
    std::lock_guard<std::mutex> lock(*mtx_);
    ready_.clear();
}

} // namespace base
} // namespace gs130
