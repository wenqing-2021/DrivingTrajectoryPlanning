#pragma once
#include "OsqpEigen/OsqpEigen.h"
#include "map/map.h"
#include "ocean_thread_pool.h"
#include "params.pb.h"
#include "problem.pb.h"
#include "vehicle_model/kinematic_model.h"
#include <Eigen/Sparse>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace planning::backend {

// Sparse assembly of a quadratic cost in the 1/2 x'Hx + g'x convention.
// square() adds one weighted squared residual, constraint() adds one row.
struct OCEANQuadratic {
    using Terms = std::vector<std::pair<int, double>>;

    explicit OCEANQuadratic(int size)
        : n(size)
        , g(Eigen::VectorXd::Zero(size)) {}

    void square(const Terms& terms, double offset, double weight);
    void bound(int i, double lo, double hi) { constraint({{i, 1}}, lo, hi); }
    void constraint(const Terms& terms, double lo, double hi);

    int                                 n;
    std::vector<Eigen::Triplet<double>> h, a;
    Eigen::VectorXd                     g;
    std::vector<double>                 lower, upper;
};

// Reuses OSQP state between ADMM iterations. reset() clears the previous
// request before the next solve initializes a new problem structure.
class OCEANQp {
  public:
    void reset();
    bool solve(const OCEANQuadratic& q);

    const Eigen::VectorXd& solution() const { return solution_; }
    int                    status() const { return status_; }

  private:
    OsqpEigen::Solver           solver_;
    Eigen::SparseMatrix<double> hessian_, constraints_;
    Eigen::VectorXd             gradient_, lower_, upper_, solution_;
    bool                        ready_  = false;
    int                         status_ = 0;
};

// One obstacle O(m) of Eq. (3) together with its dual block of Eq. (15). The
// block is independent per node, so every (obstacle, node) pair keeps its own
// certificate, its own cone buffers and its own counters.
class OCEANObstacle {
  public:
    OCEANObstacle() = default;
    OCEANObstacle(const Eigen::MatrixXd& halfspaces, const Eigen::VectorXd& bounds, int node_count);

    // Eq. (13) and (14): the two ADMM residuals of this (obstacle, node) pair.
    Eigen::Vector3d residual(const Eigen::Vector4d& state, const Eigen::Vector4d& body, int k) const;
    bool solveNode(const Eigen::MatrixXd& states, const Eigen::Vector4d& body, int node, double rho, double weight,
                   double clearance, int max_inner, double tolerance, std::chrono::steady_clock::time_point deadline);

    double          distance(int node) const { return d[node]; }
    Eigen::Vector2d normal(int node) const;

  private:
    // The ADMM owner updates the coupled primal/dual blocks directly. Other
    // callers can solve a node and inspect its certificate, but cannot edit it.
    friend class OCEANSolver;

    void reset(int node_count);
    // KKT error of the current (lambda, mu, d) for one node. With project=true
    // the point is first moved onto the feasible set of Eq. (11f)-(11i), so an
    // accepted certificate never violates its own cone.
    double certificateError(const Eigen::Vector4d& state, const Eigen::Vector4d& body, int node, double rho,
                            double weight, double clearance, bool project);

    // Per-node workspace: expensive to allocate, so it is owned across rounds.
    struct Node {
        bool                                ready     = false;   // this node holds an accepted certificate
        bool                                reused    = false;   // accepted without calling the cone solver
        bool                                projected = false;   // accepted after projecting onto the feasible set
        int                                 status = 0, iterations = 0;
        double                              error     = 0;   // KKT error of the accepted certificate
        double                              violation = 0;   // feasibility violation of the accepted certificate
        Eigen::SparseMatrix<double>         matrix, equality;
        Eigen::VectorXd                     rhs, h, c;
        Eigen::VectorXi                     cones;
        std::vector<Eigen::Triplet<double>> terms;
    };

    Eigen::MatrixXd   A;   // halfspaces A y <= b of this obstacle
    Eigen::VectorXd   b;
    Eigen::MatrixXd   lambda, mu, xi;
    Eigen::VectorXd   d, zeta;
    std::vector<Node> nodes;
};

// Owned by solver::Solver and reused across planning requests.
struct OCEANWorkspace {
    OCEANQp                     speed, path;
    OCEANThreadPool             threads;
    std::vector<OCEANObstacle>  obstacles;
    problem::PlannerDiagnostics stats;

    void reset();
};

// OCEAN: ADMM over the four variable groups of Eq. (15), (18), (19) and (20).
class OCEANSolver {
  public:
    OCEANSolver(const std::shared_ptr<vehicle_model::KinematicModel>& model, const std::shared_ptr<map::Map>& map,
                const params::OCEANParams& params, double dt, std::shared_ptr<OCEANWorkspace> workspace = nullptr);

    bool Process(const vehicle_model::sdv_path& path, std::shared_ptr<vehicle_model::VehiclePose>& start,
                 std::shared_ptr<vehicle_model::VehiclePose>& goal);

    const std::vector<Eigen::Vector4d>& GetStatesResult() const { return output_; }
    const std::vector<Eigen::Vector4d>& GetInitialStates() const { return initial_; }
    const Eigen::MatrixXd&              GetControlsResult() const { return controls_output_; }
    const Eigen::VectorXd&              GetTimeSteps() const { return h_; }
    const problem::PlannerDiagnostics&  GetStats() const { return w_->stats; }
    void                                SetTimeBudget(double seconds) { budget_ = seconds; }

    // Fill the fields an OCEANParams leaves unset; explicit zeros are kept.
    static params::OCEANParams Defaults(params::OCEANParams params);
    // Rear-axle forward-Euler step, the model OCEAN, RDA and OBCA all plan with.
    static Eigen::Vector4d Step(const Eigen::Vector4d& state, double steer, double accel, double dt, double wheelbase);

  private:
    bool   initialize(const vehicle_model::sdv_path& path, const vehicle_model::VehiclePose& start,
                      const vehicle_model::VehiclePose& goal);
    bool   solveObstacles();
    bool   solveSpeed();
    bool   solveTime();
    bool   solvePath();
    double updateResiduals();
    double objective() const;
    double merit() const;
    bool   validate() const;
    bool   fail(const std::string& reason);
    bool   expired() const;

    std::shared_ptr<vehicle_model::KinematicModel> model_;
    std::shared_ptr<map::Map>                      map_;
    params::OCEANParams                            p_;
    std::shared_ptr<OCEANWorkspace>                w_;
    Eigen::MatrixXd                                x_, reference_, eta_;   // 4 x (N+1), 4 x N
    Eigen::VectorXd                                steer_, acc_, h_;       // N
    Eigen::Vector4d                                body_;                  // front, half width, rear, half width
    Eigen::Vector2d                                origin_;
    Eigen::Vector3d                                start_, goal_;
    std::vector<Eigen::Vector4d>                   output_, initial_;
    Eigen::MatrixXd                                controls_output_;   // N x [acceleration, steering]
    double                                         dt_, budget_;
    int                                            N_ = 0;
    std::chrono::steady_clock::time_point          deadline_;
};

}   // namespace planning::backend
