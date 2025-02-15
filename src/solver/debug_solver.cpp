
#include "problem.pb.h"
#include "solver/solver.h"
#include <fstream>
#include <iostream>
#include <sstream>

std::string load_protobuf_str(const std::string& fileAddress) {
    std::ifstream file(fileAddress, std::ios::binary);
    if (!file) {
        // Handle file open error
        return "";
    }

    std::ostringstream oss;
    oss << file.rdbuf();
    std::string ret = oss.str();

    return ret;
}

int main() {
    std::cout << "start to test sovler" << std::endl;
    // 1. load protobuf
    std::string solver_input_str_path = "src/solver/debug/solver_input_str.pb";

    std::string solver_input_str = load_protobuf_str(solver_input_str_path);

    // 2. create solver and test
    problem::SolverInput solver_input;
    solver_input.ParseFromString(solver_input_str);

    solver::Solver solver;
    const auto&    plan_res = solver.Run(solver_input);

    return 0;
}