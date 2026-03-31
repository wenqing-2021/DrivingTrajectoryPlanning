"""
Load saved protobuf results and visualize with Bokeh.
"""

import argparse
import os
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
PROTOBUF_PATH = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../../install/src/protobuf"))

if SRC_ROOT not in sys.path:
    sys.path.append(SRC_ROOT)
if PROTOBUF_PATH not in sys.path:
    sys.path.append(PROTOBUF_PATH)

from protobuf.problem_pb2 import PlanProblem, PlanRes
from utils.visualization.vis_tool import BokehVis


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser("SavedResultVisualization")
    parser.add_argument(
        "--res_save_path",
        "-s",
        type=str,
        required=True,
        help="Directory containing plan_problem.pb and plan_res.pb",
    )
    parser.add_argument(
        "--problem_pb",
        type=str,
        default="plan_problem.pb",
        help="Filename of saved PlanProblem protobuf",
    )
    parser.add_argument(
        "--res_pb",
        type=str,
        default="plan_res.pb",
        help="Filename of saved PlanRes protobuf",
    )
    parser.add_argument("--debug", "-d", action="store_true")
    return parser.parse_args()


def load_pb(path: str, message_obj):
    if not os.path.exists(path):
        raise FileNotFoundError(f"protobuf file not found: {path}")
    with open(path, "rb") as f:
        message_obj.ParseFromString(f.read())


def main():
    args = parse_args()

    problem_path = os.path.join(args.res_save_path, args.problem_pb)
    result_path = os.path.join(args.res_save_path, args.res_pb)

    plan_problem = PlanProblem()
    plan_res = PlanRes()
    load_pb(problem_path, plan_problem)
    load_pb(result_path, plan_res)

    vis = BokehVis(plan_problem, plan_res, args.debug)
    vis.run()


main()
