#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>

/**
 * I2CQueue
 *
 * Thread-safe bounded job queue for serialising all I2C bus transactions
 * through a single worker thread.
 *
 * - push()          : drop-oldest on overflow  (motor / servo commands)
 * - push_critical() : never drops              (encoder reads, init)
 * - pop()           : blocks until a job is available or stop is set
 */
class I2CQueue
{
public:
    using Job = std::function<void(int /*fd*/)>;

    explicit I2CQueue(std::size_t capacity);

    /// Enqueue a job; if the queue is full the oldest pending job is discarded.
    void push(Job job);

    /// Enqueue a job that will never be discarded on overflow.
    void push_critical(Job job);

    /// Block until a job is available or stop is set.
    /// Returns false when the queue is empty AND stop is set.
    bool pop(Job &out, std::atomic<bool> &stop);

private:
    const std::size_t       capacity_;
    std::deque<Job>         queue_;
    std::mutex              mutex_;
    std::condition_variable cv_;
};
