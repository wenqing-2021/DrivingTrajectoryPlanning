#pragma once

#include "cone_thread_pool.h"
#include "eicos.hpp"
#include "qp_solver/qp_solver.h"
#include <Eigen/Sparse>
#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace planning::backend {

// Build the compressed pattern once, then accumulate directly into its values.
// Explicit zeros are retained: a coefficient crossing zero must not change CSC.
class SparseValues {
  public:
    void begin(int rows, int cols) {
        if (matrix.rows() != rows || matrix.cols() != cols) {
            matrix.resize(rows, cols);
            entries_.clear();
            offsets_.clear();
            ready_ = false;
        }
        cursor_ = 0;
        if (ready_) std::fill_n(matrix.valuePtr(), matrix.nonZeros(), 0.0);
    }
    void emplace_back(int row, int col, double value) {
        if (!ready_) {
            entries_.emplace_back(row, col, value);
        } else {
            if (cursor_ >= offsets_.size() || entries_[cursor_].row() != row || entries_[cursor_].col() != col)
                throw std::logic_error("RDA sparse structure changed without preparation");
            matrix.valuePtr()[offsets_[cursor_++]] += value;
        }
    }
    const Eigen::SparseMatrix<double>& finish() {
        if (!ready_) {
            matrix.setFromTriplets(entries_.begin(), entries_.end());
            matrix.makeCompressed();
            offsets_.reserve(entries_.size());
            for (const auto& entry : entries_) {
                const int   first = matrix.outerIndexPtr()[entry.col()];
                const int   last  = matrix.outerIndexPtr()[entry.col() + 1];
                const auto* pos =
                    std::lower_bound(matrix.innerIndexPtr() + first, matrix.innerIndexPtr() + last, entry.row());
                offsets_.push_back(pos - matrix.innerIndexPtr());
            }
            ready_ = true;
        } else if (cursor_ != offsets_.size()) {
            throw std::logic_error("Incomplete RDA sparse value update");
        }
        return matrix;
    }
    Eigen::SparseMatrix<double> matrix;

  private:
    bool                                ready_  = false;
    std::size_t                         cursor_ = 0;
    std::vector<Eigen::Triplet<double>> entries_;
    std::vector<int>                    offsets_;
};

struct ConeWorkspace {
    SparseValues                   coefficients;
    Eigen::SparseMatrix<double>    equality;
    Eigen::VectorXd                c, h, b;
    Eigen::VectorXi                cones;
    Eigen::VectorXd                result;
    EiCOS::exitcode                status = EiCOS::exitcode::fatal;

    void prepare(int T, int edges) {
        const int vars = (edges + 6) * T + 1;
        const int rows = (edges + 13) * T + 1;
        c.setZero(vars);
        c(vars - 1) = 1;
        h.setZero(rows);
        result.setZero(vars);
        equality.resize(0, vars);
        b.resize(0);
        cones.setConstant(T + 1, 3);
        cones(0) = 1 + 3 * T;
    }
    EiCOS::exitcode solve() {
        const auto& G = coefficients.matrix;
        EiCOS::Solver solver(G, equality, c, h, b, cones);
        status = solver.solve();
        result = solver.solution();
        return status;
    }
};

// Owned by the public Solver, independent of per-request map/planner lifetimes.
struct RDAWorkspace {
    struct Timings {
        double prepare_ms = 0, su_assembly_ms = 0, su_solve_ms = 0, cone_assembly_ms = 0, cone_solve_ms = 0;
        int    iterations = 0;
    } timings;
    Eigen::MatrixXd            dynamics_a, dynamics_b;
    Eigen::VectorXd            dynamics_d, dynamics_c;
    std::size_t                prepared_T = 0;
    std::vector<std::size_t>   prepared_edges;
    SparseValues               su_h, su_a;
    Eigen::VectorXd            su_g, su_lower, su_upper;
    std::unique_ptr<QPSolver>  su_solver;
    std::vector<ConeWorkspace> cones;
    ConeThreadPool             threads;
};

}   // namespace planning::backend
