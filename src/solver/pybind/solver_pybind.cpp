#include "solver/solver.h"
#include <fstream>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <vector>

namespace py = pybind11;

namespace solver {

std::shared_ptr<Solver> make_solver() {
    auto solver_ptr = std::make_shared<Solver>();

    return solver_ptr;
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
                         std::cout << "save solver input to solver_input_str.pb" << std::endl;
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
            const cost_map::CostMap& cost_map = solver.getCostMap();
            std::string              cost_map_str;
            cost_map.SerializeToString(&cost_map_str);

            return cost_map_str;
        });
}

}   // namespace solver