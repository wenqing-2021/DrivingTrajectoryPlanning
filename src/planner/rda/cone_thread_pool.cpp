#include "cone_thread_pool.h"
#include "rda_workspace.h"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <iostream>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <thread>

namespace planning::backend {
namespace {
using Clock = std::chrono::steady_clock;
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
// Balance by stored nonzeros, the dominant per-obstacle cost driver.
std::vector<std::vector<std::size_t>> partition(const std::vector<ConeWorkspace>& problems, int workers) {
    std::vector<std::size_t> order(problems.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
        return problems[a].coefficients.matrix.nonZeros() > problems[b].coefficients.matrix.nonZeros();
    });
    std::vector<std::vector<std::size_t>> assignment(workers);
    std::vector<std::size_t>              loads(workers, 0);
    for (auto index : order) {
        const auto owner = std::min_element(loads.begin(), loads.end()) - loads.begin();
        assignment[owner].push_back(index);
        loads[owner] += problems[index].coefficients.matrix.nonZeros();
    }
    return assignment;
}
}   // namespace

struct ConeThreadPool::Impl {
    std::mutex                            mutex;
    std::condition_variable               work_cv;
    std::condition_variable               done_cv;
    std::vector<std::thread>              threads;
    std::vector<std::vector<std::size_t>> assignment;
    std::vector<ConeWorkspace>*           problems      = nullptr;
    std::size_t                           problem_count = 0;
    std::vector<char>                     thread_failed;
    unsigned                              generation = 0;
    int                                   pending    = 0;
    bool                                  stop       = false;

    ~Impl() { shutdown(); }

    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        work_cv.notify_all();
        // A worker inside a solve is always joined; an abandoned thread would
        // otherwise keep writing into the workspace it points at.
        for (auto& thread : threads) {
            if (thread.joinable()) thread.join();
        }
        threads.clear();
    }

    void start(std::vector<ConeWorkspace>& items, int workers) {
        // Eigen documents initParallel() before Eigen is used from several
        // threads, and setNbThreads() writes an unsynchronized global, so both
        // run once here on the owning thread.
        Eigen::initParallel();
        Eigen::setNbThreads(1);
        problems      = &items;
        problem_count = items.size();
        assignment    = partition(items, workers);
        thread_failed.assign(workers, 0);
        threads.reserve(workers);
        for (int index = 0; index < workers; ++index) threads.emplace_back([this, index] { worker(index); });
    }

    void worker(int index) {
        // One Eigen thread per worker: the parallelism is across obstacles.
        Eigen::setNbThreads(1);
        unsigned                     seen = 0;
        std::vector<std::size_t>     mine;
        std::unique_lock<std::mutex> lock(mutex);
        for (;;) {
            work_cv.wait(lock, [&] { return stop || generation != seen; });
            if (stop) return;
            seen = generation;
            mine = assignment[index];
            lock.unlock();
            bool failed = false;
            for (auto item : mine) {
                try {
                    problems->at(item).solve();
                }
                catch (const std::exception& error) {
                    std::cerr << "RDA thread worker: " << error.what() << '\n';
                    failed = true;
                }
            }
            lock.lock();
            thread_failed[index] = failed ? 1 : 0;
            if (--pending == 0) done_cv.notify_all();
        }
    }

    bool solve(std::vector<ConeWorkspace>& items, int timeout_ms) {
        require(items.size() == problem_count && problems == &items, "RDA thread pool structure changed without reset");
        std::unique_lock<std::mutex> lock(mutex);
        pending = static_cast<int>(threads.size());
        std::fill(thread_failed.begin(), thread_failed.end(), 0);
        ++generation;
        work_cv.notify_all();
        const auto deadline  = Clock::now() + std::chrono::milliseconds(timeout_ms);
        const bool timed_out = !done_cv.wait_until(lock, deadline, [&] { return pending == 0; });
        if (timed_out) {
            // A running solve cannot be preempted from another thread. Wait for
            // the round to drain before reporting failure, so the caller never
            // observes a workspace that a worker is still writing to.
            done_cv.wait(lock, [&] { return pending == 0; });
            std::cerr << "RDA thread pool: round exceeded " << timeout_ms << " ms\n";
        }
        return !timed_out &&
               std::none_of(thread_failed.begin(), thread_failed.end(), [](char value) { return value != 0; });
    }
};

ConeThreadPool::ConeThreadPool()  = default;
ConeThreadPool::~ConeThreadPool() = default;
void ConeThreadPool::reset() {
    impl_.reset();
}

bool ConeThreadPool::solve(std::vector<ConeWorkspace>& problems, int workers, int timeout_ms) {
    try {
        if (problems.empty()) return true;
        require(workers > 0 && timeout_ms > 0, "Invalid RDA worker count or timeout");
        workers = std::min<int>(workers, problems.size());
        if (!impl_ || static_cast<int>(impl_->threads.size()) != workers || impl_->problems != &problems) {
            impl_ = std::make_unique<Impl>();
            impl_->start(problems, workers);
        }
        return impl_->solve(problems, timeout_ms);
    }
    catch (const std::exception& error) {
        std::cerr << "RDA thread pool: " << error.what() << '\n';
        reset();
        return false;
    }
}
}   // namespace planning::backend
