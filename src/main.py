"""
Author: wenqing yuansj@hnu.edu.cn
FilePath: /AutomatedPark/src/main.py
Description: The main entry of the project

Copyright (c) 2024 by wenqing, All Rights Reserved. 
"""

import argparse
import sys

PYBIND_PATH = "/root/workspace/AutomatedPark/build/pybind_modules"
PROTOBUF_PATH = "/root/workspace/AutomatedPark/build/install/protobuf"
sys.path.append(PYBIND_PATH)
sys.path.append(PROTOBUF_PATH)
import solver_pybind
from utils.plan_utils import build_problem, load_solver_params
from utils.visualization.vis_tool import BokehVis
from protobuf.problem_pb2 import PlanProblem, PlanRes, SolverInput
from protobuf.params_pb2 import SolverParams


def SolveProblem(solver, plan_problem: PlanProblem, solver_params: SolverParams):
    solver_input = SolverInput()
    solver_input.plan_problem.CopyFrom(plan_problem)
    solver_input.solver_params.CopyFrom(solver_params)
    solver_input_str = solver_input.SerializeToString()
    plan_res_str = solver.run(solver_input_str)
    plan_res = PlanRes()
    plan_res.ParseFromString(plan_res_str)

    return plan_res


def add_args():
    args = argparse.ArgumentParser("AutomatedPark")
    args.add_argument(
        "--file",
        "-f",
        type=str,
        default="/root/workspace/AutomatedPark/data/BenchmarkCases/Case1.csv",
    )
    args.add_argument(
        "--params",
        "-p",
        type=str,
        default="/root/workspace/AutomatedPark/src/config/solver_params.yaml",
    )
    args = args.parse_args()
    return args


# ======================== parse the arguments
args = add_args()

# ======================== load the case and solver params
plan_problem: PlanProblem = build_problem(args.file)
solver_params: SolverParams = load_solver_params(args.params)

# ======================== load the solver
solver = solver_pybind.make_solver()

# ======================== solve the problem
plan_res: PlanRes = SolveProblem(solver, plan_problem, solver_params)

# ======================== save the result and visualization
vis = BokehVis(plan_problem, solver_params)
vis.run()
