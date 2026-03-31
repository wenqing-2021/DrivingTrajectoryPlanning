"""
Author: wenqing yuansj@hnu.edu.cn
Description: The main entry of the project

Copyright (c) 2024 by wenqing, All Rights Reserved.
"""

import argparse
import sys
import os

# Get the directory of the current script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Construct relative paths, only works for run main in the build/install directory
PYBIND_PATH = os.path.join(SCRIPT_DIR, "../../pybind_modules")
PROTOBUF_PATH = os.path.join(SCRIPT_DIR, "../../install/src/protobuf")
sys.path.append(PYBIND_PATH)
sys.path.append(PROTOBUF_PATH)
import solver_pybind
from utils.plan_utils import build_problem, load_solver_params
from utils.visualization.vis_tool import BokehVis
from protobuf.problem_pb2 import PlanProblem, PlanRes, SolverInput
from protobuf.params_pb2 import SolverParams
from protobuf.cost_map_pb2 import CostMap

def store_solved_result(plan_problem: PlanProblem, plan_res: PlanRes, save_path: str):
    """Save plan_problem and plan_res to protobuf files."""
    if not save_path:
        return
    
    # Create directory if it doesn't exist
    os.makedirs(save_path, exist_ok=True)
    
    # Save plan_problem
    problem_file = os.path.join(save_path, "plan_problem.pb")
    with open(problem_file, "wb") as f:
        f.write(plan_problem.SerializeToString())
    
    # Save plan_res
    result_file = os.path.join(save_path, "plan_res.pb")
    with open(result_file, "wb") as f:
        f.write(plan_res.SerializeToString())
    
    print(f"Results saved to {save_path}")

def SolveProblem(
    solver,
    plan_problem: PlanProblem,
    solver_params: SolverParams,
    args: argparse.Namespace,
) -> PlanRes:
    solver_input = SolverInput()
    solver_input.plan_problem.CopyFrom(plan_problem)
    solver_input.solver_params.CopyFrom(solver_params)
    solver_input_str = solver_input.SerializeToString()
    is_debug = args.debug
    plan_res_str = solver.run(solver_input_str, is_debug)
    cost_map_str = solver.get_cost_map()
    plan_res = PlanRes()
    cost_map = CostMap()
    cost_map.ParseFromString(cost_map_str)
    plan_res.ParseFromString(plan_res_str)
    plan_res.cost_map.CopyFrom(cost_map)

    return plan_res


def add_args():
    args = argparse.ArgumentParser("TrajectoryPlanning")
    args.add_argument(
        "--file",
        "-f",
        type=str,
        default=os.path.join(SCRIPT_DIR, "../../../data/BenchmarkCases/Case2.csv"),
    )
    args.add_argument(
        "--params",
        "-p",
        type=str,
        default="config/solver_params.yaml",
    )
    args.add_argument(
        "--vehicle_yaml",
        type=str,
        default="config/vehicle_cfg.yaml",
    )
    args.add_argument(
        "--res_save_path",
        "-s",
        type=str,
        default="",
        help="Path to save plan_problem and plan_res protobuf files",
    )
    args.add_argument("--debug", "-d", action="store_true")
    args = args.parse_args()
    return args


# ======================== parse the arguments
args = add_args()

# ======================== load the case and solver params
plan_problem: PlanProblem = build_problem(args.file, args.vehicle_yaml)
solver_params: SolverParams = load_solver_params(args.params)

# ======================== load the solver
solver = solver_pybind.make_solver()

# ======================== solve the problem
plan_res: PlanRes = SolveProblem(solver, plan_problem, solver_params, args)

# ======================== save the result
store_solved_result(plan_problem, plan_res, args.res_save_path)
