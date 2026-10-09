#include "planner/obca/obca_planner.h"
#include "vehicle_model/kinematic_model.h"
#include <iostream>
#include <stdexcept>

namespace {
void Near(double value, double expected, double tolerance = 1e-8) {
    if (std::abs(value - expected) > tolerance) throw std::runtime_error("Derivative check failed");
}
}   // namespace

int main() {
    using Model = planning::vehicle_model::KinematicModel;
    try {
        // Linear/quadratic signals on an uneven grid have known derivatives.
        const double h0 = 0.08, h1 = 0.17;
        Near(Model::ControlRate(h0)(0, 3 * h0), 3 * std::sqrt(h0));
        Near(Model::ControlAcceleration(h0, h1)(1, 1 + 2 * h0, 1 + 2 * (h0 + h1)), 0);
        Near(Model::ControlAcceleration(h0, h1)(0, h0 * h0, (h0 + h1) * (h0 + h1)), 2 * std::sqrt((h0 + h1) / 2));
        // Numerical differentiation independently checks time Jacobians.
        const double u0 = 0.1, u1 = -0.2, u2 = 0.4, epsilon = 1e-6;
        Near(Model::ControlRateTimeJacobian(u0, u1, h0),
             (Model::ControlRate(h0 + epsilon)(u0, u1) - Model::ControlRate(h0 - epsilon)(u0, u1)) / (2 * epsilon),
             1e-6);
        const auto jac = Model::ControlAccelerationTimeJacobian(u0, u1, u2, h0, h1);
        Near(jac[0],
             (Model::ControlAcceleration(h0 + epsilon, h1)(u0, u1, u2) -
              Model::ControlAcceleration(h0 - epsilon, h1)(u0, u1, u2)) /
                 (2 * epsilon),
             1e-5);
        Near(jac[1],
             (Model::ControlAcceleration(h0, h1 + epsilon)(u0, u1, u2) -
              Model::ControlAcceleration(h0, h1 - epsilon)(u0, u1, u2)) /
                 (2 * epsilon),
             1e-5);

        // Exercise OBCA's actual AD objective at a non-default dt. Separate
        // weights and a dummy terminal control must not contaminate each other.
        using namespace planning::backend;
        using namespace planning;
        OBCAFG_eval fg;
        auto        ref   = std::make_shared<Eigen::VectorXd>(Eigen::VectorXd::Zero(24));
        auto        model = std::make_shared<vehicle_model::KinematicModel>(kinematic_model::VehicleParam());
        auto        start = std::make_shared<vehicle_model::VehiclePose>();
        auto        goal  = std::make_shared<vehicle_model::VehiclePose>();
        fg.setInitParameters(ref, 4, 4, 2, 0, 0, model, start, goal);
        fg.setTimeStep(0.2);
        FG_eval::ADvector x(24);
        for (std::size_t i = 0; i < x.size(); ++i) x[i] = 0;
        x[ACCELERATION]      = 0;
        x[6 + ACCELERATION]  = 0.6;
        x[12 + ACCELERATION] = 1.2;
        x[STEER_ANGLE]       = 0;
        x[6 + STEER_ANGLE]   = 0.04;
        x[12 + STEER_ANGLE]  = 0.16;
        x[18 + STEER_ANGLE]  = 100;
        x[18 + ACCELERATION] = 100;
        params::OBCAParams p;
        p.mutable_smoothness()->set_jerk_weight(1);
        fg.setCostWeights(p);
        Near(CppAD::Value(fg.getCostFunction(x)), 3.6);
        p.mutable_smoothness()->set_jerk_weight(0);
        p.mutable_smoothness()->set_steering_acceleration_weight(1);
        fg.setCostWeights(p);
        Near(CppAD::Value(fg.getCostFunction(x)), 0.8);
        p.mutable_smoothness()->set_steering_acceleration_weight(0);
        fg.setCostWeights(p);
        Near(CppAD::Value(fg.getCostFunction(x)), 0);
        p.mutable_smoothness()->set_jerk_weight(-1);
        bool rejected = false;
        try {
            Model::ValidateSmoothness(p.smoothness());
        }
        catch (const std::invalid_argument&) {
            rejected = true;
        }
        if (!rejected) throw std::runtime_error("Negative weight accepted");
        std::cout << "Control derivative and OBCA objective checks passed\n";
    }
    catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
