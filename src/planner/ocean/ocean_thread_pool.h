#pragma once
#include <functional>
#include <memory>
#include <vector>

namespace planning::backend {
// Persistent CPU workers for the obstacle block of Eq. (15). Tasks pull their
// index from an atomic counter, the pool never shares a solver between threads,
// and a round owns its captures until every task has finished, including the
// tasks still running when the watchdog fires.
class OCEANThreadPool {
  public:
    OCEANThreadPool();
    ~OCEANThreadPool();
    OCEANThreadPool(const OCEANThreadPool&)            = delete;
    OCEANThreadPool& operator=(const OCEANThreadPool&) = delete;

    bool run(int count, int workers, int timeout_ms, const std::function<bool(int)>& task);

  private:
    struct State;
    std::unique_ptr<State> state_;
};
}   // namespace planning::backend
