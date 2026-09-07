/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * 
 * Software Fifo, Thread-safe, header-only
 */
#ifndef GS130_BASE_FIFO_HPP
#define GS130_BASE_FIFO_HPP

#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

#include "types.hpp"

namespace gs130 {
namespace base {


template <typename T>
class Fifo {
public:
    Fifo(size_t depth, FifoMode mode): 
        mode_(mode), buf_(depth), valid_(depth >= 2),
        mtx_(std::make_unique<std::mutex>())
    {    
    }

    // copying disabled
    Fifo(const Fifo &) = delete;
    Fifo &operator=(const Fifo &) = delete;

    // define move behavior
    Fifo(Fifo &&other) noexcept: 
        mode_(other.mode_), buf_(std::move(other.buf_)),
        head_(other.head_), tail_(other.tail_), count_(other.count_),
        valid_(other.valid_), mtx_(std::move(other.mtx_))
    {
        other.valid_ = false;
    }
    Fifo &operator=(Fifo &&other) noexcept
    {
        if(this != &other){
            mode_  = other.mode_;
            buf_   = std::move(other.buf_);
            head_  = other.head_;
            tail_  = other.tail_;
            count_ = other.count_;
            valid_ = other.valid_;
            mtx_   = std::move(other.mtx_);
            other.valid_ = false;
        }
        return *this;
    }

    explicit operator bool() const { return valid_; }

    bool push(const T &item)
    {
        if(!valid_) return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(full_unlocked()){
            if(mode_ == FifoMode::DropNew)
                return false;
            tail_ = (tail_ + 1) % buf_.size();   // overwrite the oldest
        }
        else{
            ++count_;
        }
        buf_[head_] = item;
        head_ = (head_ + 1) % buf_.size();
        return true;
    }

    bool pop(T &item)
    {
        if(!valid_) return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        if(count_ == 0)
            return false;
        item = buf_[tail_];
        tail_ = (tail_ + 1) % buf_.size();
        --count_;
        return true;
    }

    size_t size() const
    {
        if(!valid_) return 0;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_;
    }

    bool empty() const
    {
        if(!valid_) return true;
        std::lock_guard<std::mutex> lock(*mtx_);
        return count_ == 0;
    }

    bool full() const
    {
        if(!valid_) return false;
        std::lock_guard<std::mutex> lock(*mtx_);
        return full_unlocked();
    }

    size_t capacity() const { return buf_.size(); }

private:
    bool full_unlocked() const { return count_ == buf_.size(); }

    FifoMode                 mode_;
    std::vector<T>           buf_;
    size_t              head_  = 0;
    size_t              tail_  = 0;
    size_t              count_ = 0;
    bool                     valid_;
    std::unique_ptr<std::mutex> mtx_;
};

} // namespace base
} // namespace gs130

#endif // GS130_BASE_FIFO_HPP