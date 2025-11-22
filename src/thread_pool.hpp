#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <semaphore>
#include <functional>
#include <condition_variable>
#include <iostream>

class WaitGroup {
private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::size_t count_ = 0;
public:
    WaitGroup() noexcept = default;
    void add() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        count_++;
    }
    void add(std::size_t n) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        count_ += n;
    }
    void done() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (--count_ == 0) {
            cv_.notify_all();
        }
    }
    void wait() noexcept {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return count_ == 0; });
    }
};

template<typename T, std::size_t Capacity>
class CircularBuffer {
private:
    std::aligned_storage_t<sizeof(T), alignof(T)> buffer_[Capacity];
    std::size_t head_ = 0;
    std::size_t tail_ = 0;
    std::size_t size_ = 0;
public:
    constexpr CircularBuffer() noexcept = default;
    constexpr ~CircularBuffer() noexcept {
        while (size_ > 0) {
            pop();
        }
    }
    void push(const std::function<void()>& task) noexcept {
        assert(size_ < Capacity && "Buffer overflow");
        new (&buffer_[tail_]) T(task);
        tail_ = (tail_ + 1) % Capacity;
        size_++;
    }
    T pop() noexcept {
        assert(size_ > 0 && "Buffer underflow");
        auto task = std::move(*std::launder(reinterpret_cast<T*>(&buffer_[head_])));
        head_ = (head_ + 1) % Capacity;
        size_--;
        return task;
    }
    std::size_t size() const noexcept {
        return size_;
    }
};

template<std::size_t QueueSize = 1024>
class ThreadPool {
private:
    CircularBuffer<std::function<void()>, QueueSize> tasks_;
    std::vector<std::jthread> workers_;
    std::mutex queue_mutex_;
    std::counting_semaphore<QueueSize> task_semaphore_ = std::counting_semaphore<QueueSize>(0);
    std::counting_semaphore<QueueSize> slot_semaphore_ = std::counting_semaphore<QueueSize>(QueueSize);
    WaitGroup idle_wait_group_;
    std::stop_source stop_source_;
public:
    ThreadPool(std::size_t max_threads = std::thread::hardware_concurrency()) noexcept {
        assert(max_threads > 0);
        workers_.reserve(max_threads);
        for (std::size_t i = 0; i < max_threads; i++) {
            workers_.emplace_back([this]() { worker_thread(stop_source_.get_token()); });
        }
    };
    ~ThreadPool() noexcept {
        stop_source_.request_stop();
        for (std::size_t i = 0; i < workers_.size(); i++) {
            task_semaphore_.release();
        }
    }
    void enqueue(std::function<void()> task) noexcept {
        slot_semaphore_.acquire();
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            tasks_.push(std::move(task));
        }
        idle_wait_group_.add();
        task_semaphore_.release();
    }
    void wait_idle() noexcept {
        idle_wait_group_.wait();
    }
private:
    void worker_thread(std::stop_token stop_token) noexcept {
        while (true) {
            task_semaphore_.acquire();
            if (stop_token.stop_requested()) {
                break;
            }
            std::function<void()> task;
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                task = tasks_.pop();
            }
            slot_semaphore_.release();
            try {
                task();
            } catch (const std::exception& e) {
            }
            idle_wait_group_.done();
        }
    }
};