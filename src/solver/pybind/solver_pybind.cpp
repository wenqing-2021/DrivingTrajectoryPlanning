/*
 * @Author: wenqing-2021 yuansj@hnu.edu.cn
 * @Date: 2025-05-17 00:28:52
 * @LastEditors: wenqing-2021 yuansj@hnu.edu.cn
 * @LastEditTime: 2026-03-22 03:27:03
 * @FilePath: /AutomatedPark/src/solver/pybind/solver_pybind.cpp
 * @Description: pybind for c++ and python interface
 */
#include "collision_check/gjk_check.h"
#include "solver/solver.h"
#include <filesystem>
#include <fstream>
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>

namespace py = pybind11;

namespace solver {

std::shared_ptr<Solver> make_solver() {
    auto solver_ptr = std::make_shared<Solver>();

    return solver_ptr;
}

bool TestCollisionChecker(const std::vector<std::vector<double>>& obs_polygon, const std::vector<double>& vehicle_pose,
                          const std::string& vehicle_param_str) {
    // 1. generate vehicle_param
    kinematic_model::VehicleParam vehicle_param;
    vehicle_param.ParseFromString(vehicle_param_str);
    // 2. create GJKCheck instance
    const auto gjk_check = collision_check::BaseCheck::CreateChecker<collision_check::GJKCheck>(vehicle_param);
    // 3. create polygon1 and polygon2
    std::vector<common::math::Vec2d> polygon_points;
    for (const auto& point : obs_polygon) {
        common::math::Vec2d point_obj(point[0], point[1]);
        polygon_points.push_back(point_obj);
    }
    common::math::Polygon2d polygon_obj(polygon_points);
    common::math::Pose      vehicle_pose_obj(vehicle_pose[0], vehicle_pose[1], vehicle_pose[2]);
    return gjk_check->Check(polygon_obj, vehicle_pose_obj);
}

// 定义一个Pybind11模块
PYBIND11_MODULE(solver_pybind, m) {
    // 绑定 solver 类
    m.def(
        "make_solver", &make_solver, "create a solver instance and return a shared_ptr", py::return_value_policy::move);
    py::class_<Solver, std::shared_ptr<Solver>>(m, "Solver")
        .def(py::init<>())
        .def("run",
             [](Solver& solver, const std::string& solver_input_str, bool is_debug = false) -> py::bytes {
                 // process
                 problem::SolverInput solver_input;
                 if (is_debug) {
                     // Ensure the directory exists
                     std::string           save_dir = "src/solver/debug";
                     std::filesystem::path dir(save_dir);
                     if (!std::filesystem::exists(dir)) { std::filesystem::create_directories(dir); }
                     // Save the solver input
                     std::ofstream file(save_dir + "/solver_input_str.pb", std::ios::binary);
                     if (file.is_open()) {
                         file << solver_input_str;
                         file.close();
                         std::cout << "save solver input to " << save_dir << "/solver_input_str.pb" << std::endl;
                     } else {
                         std::cerr << "Failed to open file src/solver/debug/solver_input_str.pb for writing"
                                   << std::endl;
                     }
                 }
                 solver_input.ParseFromString(solver_input_str);
                 const problem::PlanRes& plan_res = solver.Run(solver_input);

                 std::string plan_res_str;
                 plan_res.SerializeToString(&plan_res_str);

                 return plan_res_str;
             })
        .def("get_cost_map", [](Solver& solver) -> py::bytes {
            // process
            const cost_map::CostMap& cost_map = solver.GetCostMap();
            std::string              cost_map_str;
            cost_map.SerializeToString(&cost_map_str);

            return cost_map_str;
        });

    // bind the collsion check
    m.def("TestCollisionChecker",
          &TestCollisionChecker,
          "Test the collision checker",
          py::arg("obs_polygon"),
          py::arg("vehicle_pose"),
          py::arg("vehicle_param_str"));
}

}   // namespace solver