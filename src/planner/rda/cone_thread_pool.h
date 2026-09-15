#pragma once
#include <memory>
#include <vector>

namespace planning::backend {
struct ConeWorkspace;
// Persistent worker threads. Each thread owns a disjoint subset of the obstacle
// SOCPs, so solver state is never shared and no marshalling is needed.
class ConeThreadPool {
  public:
    ConeThreadPool();
    ~ConeThreadPool();
    ConeThreadPool(const ConeThreadPool&)            = delete;
    ConeThreadPool& operator=(const ConeThreadPool&) = delete;
    bool            solve(std::vector<ConeWorkspace>& problems, int workers, int timeout_ms);
    void            reset();

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}   // namespace planning::backend
