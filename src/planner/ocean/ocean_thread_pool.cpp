#include "ocean_thread_pool.h"

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace planning::backend {
struct OCEANThreadPool::State {
    std::mutex               mutex;
    std::condition_variable  work, done;
    std::vector<std::thread> threads;
    std::vector<char>        success;
    std::function<bool(int)> task;
    std::atomic<int>         next{0};
    int                      count = 0, pending = 0;
    unsigned                 generation = 0;
    bool                     stop       = false;
    explicit State(int workers) {
        try {
            for (int i = 0; i < workers; ++i)
                threads.emplace_back([this, i] {
                    unsigned                     seen = 0;
                    std::unique_lock<std::mutex> lock(mutex);
                    for (;;) {
                        work.wait(lock, [&] { return stop || generation != seen; });
                        if (stop) return;
                        seen = generation;
                        lock.unlock();
                        bool ok = true;
                        for (;;) {
                            const int index = next.fetch_add(1);
                            if (index >= count) break;
                            try {
                                if (!task(index)) ok = false;
                            }
                            catch (...) {
                                ok = false;
                            }
                        }
                        lock.lock();
                        success[i] = ok;
                        if (--pending == 0) done.notify_all();
                    }
                });
        }
        catch (...) {
            shutdown();
            throw;
        }
    }
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        work.notify_all();
        for (auto& thread : threads)
            if (thread.joinable()) thread.join();
    }
    ~State() { shutdown(); }
};
OCEANThreadPool::OCEANThreadPool()  = default;
OCEANThreadPool::~OCEANThreadPool() = default;
bool OCEANThreadPool::run(int count, int workers, int timeout_ms, const std::function<bool(int)>& task) {
    if (count == 0) return true;
    if (workers <= 1) {
        // Inline execution has no worker to unblock, so the per-round watchdog
        // would only reject the round for being long. The caller's own global
        // deadline still bounds the work.
        for (int i = 0; i < count; ++i)
            if (!task(i)) return false;
        return true;
    }
    workers = std::min(count, workers);
    static std::once_flag eigen_once;
    std::call_once(eigen_once, [] {
        Eigen::initParallel();
        Eigen::setNbThreads(1);
    });
    if (!state_ || state_->threads.size() != static_cast<std::size_t>(workers))
        state_ = std::make_unique<State>(workers);
    auto&                        s = *state_;
    std::unique_lock<std::mutex> lock(s.mutex);
    s.count   = count;
    s.next    = 0;
    s.task    = task;
    s.pending = workers;
    s.success.assign(workers, 1);
    ++s.generation;
    s.work.notify_all();
    const bool in_time = s.done.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return s.pending == 0; });
    if (!in_time) s.done.wait(lock, [&] { return s.pending == 0; });
    s.task = {};
    return in_time && std::all_of(s.success.begin(), s.success.end(), [](char v) { return v != 0; });
}
}   // namespace planning::backend
