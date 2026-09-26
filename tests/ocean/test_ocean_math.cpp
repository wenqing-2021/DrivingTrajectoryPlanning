// Units the OCEAN planner is built from: the shared math helpers it relies on
// (src/common/math), its own rear-axle step, and the small pieces of the planner
// itself: the obstacle certificate, the reusable QP and the worker pool.
#include "planner/ocean/ocean_planner.h"

#include "math/math_utils.h"
#include "math/polygon2d.h"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace planning::backend;

namespace {
void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

// One (obstacle, node) problem per call is enough for the checks below.
bool SolveObstacle(OCEANObstacle& obstacle, const Eigen::MatrixXd& states, const Eigen::Vector4d& body) {
    return obstacle.solveNode(
        states, body, 0, 1000, 0.1, 0, 100, 1e-5, std::chrono::steady_clock::now() + std::chrono::seconds(10));
}
}   // namespace

int main() {
    try {
        // --- shared math ------------------------------------------------------ //
        for (double at = -10; at <= 0; at += 0.5)
            for (double d = -20; d <= 0; d += 0.2)
                Require(common::math::ExponentialUpperBound(d, at) + 1e-12 >= std::exp(d), "Exponential upper bound");
        Require(std::abs(common::math::OptimalTimeStep(2, 0, 4, 0.1, 3) - 1) < 1e-12, "Time block optimum");
        Require(common::math::OptimalTimeStep(0, 1, 0, 0.1, 3) == 0.1, "Time lower bound");
        Require(common::math::OptimalTimeStep(0, -1, 0, 0.1, 3) == 3, "Time upper bound");
        for (double offset : {-5.0, -0.01, 0.2, 2.0, 50.0}) {
            const double d = common::math::OptimalDistance(offset, 1000, 0.1, -0.1);
            Require(d <= -0.1, "Distance upper bound");
            const double gradient = 1000 * (d + offset) + 0.1 * std::exp(d);
            Require(d == -0.1 ? gradient <= 0 : std::abs(gradient) < 1e-7, "Distance KKT");
        }
        const common::math::Polygon2d concave(std::vector<common::math::Vec2d>{{0, 0}, {3, 0}, {1, 1}, {0, 3}});
        const auto                    pieces = concave.DecomposeConvex();
        Require(pieces.size() == 2, "Concave decomposition");
        double area = 0;
        for (const auto& piece : pieces) {
            Require(piece.is_convex(), "Convex piece");
            area += piece.area();
        }
        Require(std::abs(area - concave.area()) < 1e-10, "Obstacle area preserved");

        // --- planner units --------------------------------------------------- //
        const Eigen::Vector4d reverse(0, 0, 0, -2);
        Require((OCEANSolver::Step(reverse, 0, 0, 0.1, 2.5) - Eigen::Vector4d(-0.2, 0, 0, -2)).norm() < 1e-12,
                "Reverse dynamics");
        params::OCEANParams p;
        p.set_min_clearance(0);
        p.set_endpoint_position_tolerance(0);
        p = OCEANSolver::Defaults(p);
        Require(p.min_clearance() == 0 && p.endpoint_position_tolerance() == 0, "Optional explicit zero");

        // A unit body box 2 m away from the obstacle [3, 4] x [-1, 1] separates by
        // d = -2; the certificate must report that and stay inside the dual cone.
        Eigen::Matrix<double, 4, 2> halfspaces;
        halfspaces << 1, 0, 0, 1, -1, 0, 0, -1;
        const Eigen::Vector4d bounds(4, 1, -3, 1);
        OCEANObstacle         obstacle(halfspaces, bounds, 1);
        const Eigen::MatrixXd states = Eigen::MatrixXd::Zero(4, 1);
        const Eigen::Vector4d body(1, 1, 1, 1);
        Require(SolveObstacle(obstacle, states, body), "Obstacle convergence");
        Require(std::abs(obstacle.distance(0) + 2) < 2e-3, "Rectangle distance certificate sign");
        Require(obstacle.residual(Eigen::Vector4d::Zero(), body, 0).norm() < 2e-3, "Collision residual");
        Require(obstacle.normal(0).norm() <= 1 + 1e-6, "Dual cone bound");

        // The same configuration moved and rotated by a rigid transform gives the
        // same separation distance.
        const double          angle = 0.71;
        const Eigen::Vector2d shift(800, -500);
        const Eigen::MatrixXd rotated = halfspaces * common::math::RotationMatrix2d(angle).transpose();
        OCEANObstacle         moved(rotated, bounds + rotated * shift, 1);
        Eigen::MatrixXd       shifted = states;
        shifted.col(0) << shift[0], shift[1], angle, 0;
        Require(SolveObstacle(moved, shifted, body), "Rigid transform solve");
        Require(std::abs(moved.distance(0) - obstacle.distance(0)) < 1e-5, "Rigid transform certificate");

        // One workspace reused across different problem sizes must rebuild itself.
        OCEANQp workspace;
        for (int n : {2, 3, 2}) {
            workspace.reset();
            OCEANQuadratic q(n);
            for (int i = 0; i < n; ++i) {
                q.square({{i, 1}}, -1, 1);
                q.bound(i, 0, 2);
            }
            Require(workspace.solve(q), "QP reset on changed dimensions");
            Require((workspace.solution() - Eigen::VectorXd::Ones(n)).norm() < 1e-6, "QP solution");
        }

        // Worker pool contract: every task once, exceptions reported, a timed out
        // round drained, and the pool usable afterwards.
        OCEANThreadPool  pool;
        std::vector<int> output(20, 0);
        Require(pool.run(20,
                         4,
                         1000,
                         [&](int i) {
                             output[i] = i + 1;
                             return true;
                         }),
                "Thread solve");
        for (int i = 0; i < 20; ++i) Require(output[i] == i + 1, "Thread task delivered exactly once");
        Require(!pool.run(4,
                          2,
                          1000,
                          [](int i) {
                              if (i == 1) throw std::runtime_error("expected");
                              return true;
                          }),
                "Thread exception");
        Require(!pool.run(4,
                          2,
                          1,
                          [](int) {
                              std::this_thread::sleep_for(std::chrono::milliseconds(5));
                              return true;
                          }),
                "Timeout drain");
        Require(pool.run(1, 2, 1000, [](int) { return true; }), "Reuse after timeout");

        std::cout << "OCEAN planner unit checks passed\n";
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
