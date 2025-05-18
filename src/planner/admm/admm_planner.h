#include <Eigen/Core>

namespace planning {
namespace backend {
class ADMMSolver {
  public:
    ADMMSolver();
    ~ADMMSolver() = default;
    bool Solve();

  private:
};
}   // namespace backend
}   // namespace planning
