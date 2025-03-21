#include "logger/logger.h"
#include "params.pb.h"
#include <Eigen/Core>
#include <vector>
namespace planning {
namespace backend {
class MPCPlanner {
    MPCPlanner(const params::MPCSolverParams& mpc_solver_params);
    ~MPCPlanner() = default;
    void Optimization();

  public:
    // void Init(const MPCPlannerConfig& config);
    void BuildObjective();
    void BuildConstraints();

  private:
    params::MPCSolverParams mpc_solver_params_;
};
}   // namespace backend
}   // namespace planning
