#include "pincer_rover_base/i2c_queue.hpp"


I2CQueue::I2CQueue(std::size_t capacity)
: capacity_(capacity)
{}

void I2CQueue::push(Job job)
{
    std::lock_guard<std::mutex> lk(mutex_);
    if (queue_.size() >= capacity_) {
        queue_.pop_front();     // discard oldest stale command
    }
    queue_.push_back(std::move(job));
    cv_.notify_one();
}

void I2CQueue::push_critical(Job job)
{
    std::lock_guard<std::mutex> lk(mutex_);
    queue_.push_back(std::move(job));   // never discard
    cv_.notify_one();
}

bool I2CQueue::pop(Job &out, std::atomic<bool> &stop)
{
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_for(lk, std::chrono::milliseconds(100),
                 [&]{ return !queue_.empty() || stop.load(); });

    if (queue_.empty()) return false;   // covers both timeout and stop

    out = std::move(queue_.front());
    queue_.pop_front();
    return true;
}
