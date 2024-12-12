#include "solver.h"
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
        .def("run", [](Solver& solver, const std::string& solver_input_str) -> py::bytes {
            // process
            problem::SolverInput solver_input;
            solver_input.ParseFromString(solver_input_str);
            const problem::PlanRes& plan_res = solver.run(solver_input);

            std::string plan_res_str;
            plan_res.SerializeToString(&plan_res_str);

            return plan_res_str;
        });
}

}   // namespace solver