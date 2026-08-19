"""Command-line entry point for the existing parking scenario."""

import argparse
import sys
from pathlib import Path
from typing import Optional, Sequence

from protobuf.cost_map_pb2 import CostMap
from protobuf.params_pb2 import SolverParams
from protobuf.problem_pb2 import PlanProblem, PlanRes, SolverInput
from utils.plan_utils import build_problem, load_solver_params


def _load_solver_module():
    """Load the parking pybind11 module in source and install layouts."""
    try:
        import solver_pybind

        return solver_pybind
    except ImportError:
        for parent in Path(__file__).resolve().parents:
            for candidate in (
                parent / "build/pybind_modules",
                parent / "pybind_modules",
            ):
                if candidate.is_dir():
                    sys.path.insert(0, str(candidate))
                    try:
                        import solver_pybind

                        return solver_pybind
                    except ImportError:
                        continue
        raise


def store_solved_result(
    plan_problem: PlanProblem, plan_result: PlanRes, save_path: str
) -> None:
    """Store a parking problem and its solution as protobuf files."""
    if not save_path:
        return

    output_directory = Path(save_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    (output_directory / "plan_problem.pb").write_bytes(plan_problem.SerializeToString())
    (output_directory / "plan_res.pb").write_bytes(plan_result.SerializeToString())
    print(f"Results saved to {output_directory}")


def solve_problem(
    solver: object,
    plan_problem: PlanProblem,
    solver_params: SolverParams,
    debug: bool = False,
) -> PlanRes:
    """Run the C++ parking solver and decode its protobuf response."""
    solver_input = SolverInput()
    solver_input.plan_problem.CopyFrom(plan_problem)
    solver_input.solver_params.CopyFrom(solver_params)

    result_payload = solver.run(solver_input.SerializeToString(), debug)
    cost_map_payload = solver.get_cost_map()

    plan_result = PlanRes()
    plan_result.ParseFromString(result_payload)
    cost_map = CostMap()
    cost_map.ParseFromString(cost_map_payload)
    plan_result.cost_map.CopyFrom(cost_map)
    return plan_result


def _default_case_path() -> Path:
    """Find the default parking benchmark in source and install layouts."""
    relative_path = Path("data/BenchmarkCases/Case2.csv")
    for parent in Path(__file__).resolve().parents:
        candidate = parent / relative_path
        if candidate.is_file():
            return candidate
    return relative_path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """Parse parking scenario arguments."""
    parser = argparse.ArgumentParser("TrajectoryPlanning park scenario")
    parser.add_argument(
        "--file",
        "-f",
        default=str(_default_case_path()),
        help="Parking benchmark CSV file.",
    )
    parser.add_argument(
        "--params",
        "-p",
        default="config/solver_params.yaml",
        help="Solver parameter YAML file.",
    )
    parser.add_argument(
        "--vehicle_yaml",
        default="config/vehicle_cfg.yaml",
        help="Vehicle parameter YAML file.",
    )
    parser.add_argument(
        "--res_save_path",
        "-s",
        default="",
        help="Directory for the result protobuf files.",
    )
    parser.add_argument("--debug", "-d", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run one parking planning problem."""
    args = parse_args(argv)
    plan_problem = build_problem(args.file, args.vehicle_yaml)
    solver_params = load_solver_params(args.params)
    solver = _load_solver_module().make_solver()
    plan_result = solve_problem(solver, plan_problem, solver_params, args.debug)
    store_solved_result(plan_problem, plan_result, args.res_save_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
