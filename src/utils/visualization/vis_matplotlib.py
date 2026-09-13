"""
Load saved protobuf results and visualize with Matplotlib.
Saves static images to the same scenario directory as the protobuf results
(solve_results/park/场景_<solve time> by default).
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend for saving to disk
import matplotlib.pyplot as plt
from typing import Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../.."))
# protobuf generated .py files are flat modules (not a package) under build/install/src/protobuf/
# NOTE: "protobuf" is a namespace package that shadows the local protobuf/ directory,
# so we import the pb2 modules directly instead of using `from protobuf.xxx import ...`.
from pathlib import Path

if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)
for parent in Path(__file__).resolve().parents:
    for candidate in (parent / "protobuf", parent / "build/install/src/protobuf"):
        if (candidate / "problem_pb2.py").is_file():
            sys.path.insert(0, str(candidate))
            break
    else:
        continue
    break

import problem_pb2 as _problem_pb2
import kinematic_model_pb2 as _kinematic_model_pb2
import cost_map_pb2 as _cost_map_pb2

PlanProblem = _problem_pb2.PlanProblem
PlanRes = _problem_pb2.PlanRes
StateVar = _kinematic_model_pb2.StateVar
ControlVar = _kinematic_model_pb2.ControlVar
CostMap = _cost_map_pb2.CostMap


from utils.visualization.model import convert_rear_to_mid
from utils.visualization.style import (
    RENDER, FIGURE_SIZE_SMALL, FIGURE_SIZE_ESDF, SAVE_DPI,
)


# ---------------------------------------------------------------------------
# Main matplotlib-based visualiser
# ---------------------------------------------------------------------------
class MatplotlibVis:
    """Load PlanProblem / PlanRes protobufs and render static images with matplotlib."""

    def __init__(
        self,
        plan_problem: PlanProblem,
        plan_res: PlanRes,
        save_dir: str,
        debug: bool = False,
    ):
        self.plan_problem = plan_problem
        self.plan_res = plan_res
        self.save_dir = save_dir
        self.debug = debug

        os.makedirs(self.save_dir, exist_ok=True)

        # parsed data containers (mirrors vis_tool.BokehVis)
        self.init_traj_dict: dict = {}
        self.init_controls_dict: dict = {}
        self.pre_opt_traj_dict: dict = {}
        self.pre_opt_controls_dict: dict = {}
        self.obs_polygons: list = []  # list of (xs, ys)
        self.init_mid: tuple = (0.0, 0.0)
        self.goal_mid: tuple = (0.0, 0.0)
        self.safe_dis_map: Optional[np.ndarray] = None
        self.occ_pts: Optional[np.ndarray] = None   # shape (N, 2) world coords
        self.occ_mask: Optional[np.ndarray] = None   # shape (N,) bool

        # parse
        self._build_plan_problem()
        self._parse_plan_result()

    # ------------------------------------------------------------------
    #  helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _calc_axis_margin(values: np.ndarray, pad: float = 0.1):
        v = np.asarray(values, dtype=float)
        v = v[np.isfinite(v)]
        if v.size == 0:
            return -1.0, 1.0
        lo, hi = float(np.min(v)), float(np.max(v))
        span = hi - lo
        if span < 1e-9:
            span = max(abs(hi) * pad, 1e-2)
        return lo - span * pad, hi + span * pad

    # ------------------------------------------------------------------
    #  parse plan problem
    # ------------------------------------------------------------------
    def _build_plan_problem(self):
        pp = self.plan_problem

        # obstacles
        self.obs_polygons = []
        for i in range(pp.obstacle_num):
            poly = pp.obstacle_list[i]
            xs = [poly.vertex_pts[j].x for j in range(poly.vertex_num)]
            ys = [poly.vertex_pts[j].y for j in range(poly.vertex_num)]
            self.obs_polygons.append((xs, ys))

        # initial / goal mid points (used for rectangles)
        self.init_mid = convert_rear_to_mid(
            [pp.init_state.x, pp.init_state.y],
            pp.vehicle_param.rear_overhang,
            pp.vehicle_param.length,
            pp.init_state.theta,
        )
        self.goal_mid = convert_rear_to_mid(
            [pp.goal_state.x, pp.goal_state.y],
            pp.vehicle_param.rear_overhang,
            pp.vehicle_param.length,
            pp.goal_state.theta,
        )

    # ------------------------------------------------------------------
    #  parse plan result
    # ------------------------------------------------------------------
    def _parse_plan_result(self):
        pr = self.plan_res
        is_success = pr.solve_success

        # ---- ESDF map -------------------------------------------------
        cost_map: CostMap = pr.cost_map
        nx = cost_map.map_info.x_size
        ny = cost_map.map_info.y_size
        safe_dis = np.full((nx, ny), np.nan, dtype=float)
        occ_pts = np.full((nx, ny, 2), np.nan, dtype=float)
        occ_flag = np.zeros((nx, ny), dtype=bool)

        for pt in cost_map.points:
            ix = pt.index.idx_x
            iy = pt.index.idx_y
            safe_dis[ix, iy] = pt.safe_dis
            occ_pts[ix, iy, 0] = pt.position.x
            occ_pts[ix, iy, 1] = pt.position.y
            occ_flag[ix, iy] = pt.is_occupy

        self.safe_dis_map = safe_dis
        self.occ_pts_2d = occ_pts.reshape(-1, 2)
        self.occ_mask = occ_flag.reshape(-1)

        # ---- init trajectory (optimised) ------------------------------
        state_num = len(StateVar.DESCRIPTOR.fields)
        init_traj = pr.init_traj
        if len(init_traj) > 0:
            arr = np.zeros((len(init_traj), state_num))
            for idx, s in enumerate(init_traj):
                arr[idx] = [s.x, s.y, s.theta, s.v]
            self.init_traj_dict = {
                "t": np.arange(len(init_traj)),
                "x": arr[:, 0], "y": arr[:, 1],
                "head": arr[:, 2], "v": arr[:, 3],
            }

        # ---- init controls (optimised) --------------------------------
        if is_success:
            ctrl_num = len(ControlVar.DESCRIPTOR.fields)
            controls = pr.init_controls
            if len(controls) > 0:
                arr = np.zeros((len(controls), ctrl_num))
                for idx, c in enumerate(controls):
                    arr[idx] = [c.accelerate, c.steer_angle]
                self.init_controls_dict = {
                    "t": np.arange(len(controls)),
                    "acc": arr[:, 0], "steer": arr[:, 1],
                }

        # ---- pre-opt trajectory (hybrid A*) --------------------------
        pre_traj = pr.pre_opt_traj
        if is_success and len(pre_traj) > 0:
            arr = np.zeros((len(pre_traj), state_num))
            for idx, s in enumerate(pre_traj):
                arr[idx] = [s.x, s.y, s.theta, s.v]
            self.pre_opt_traj_dict = {
                "t": np.arange(len(pre_traj)),
                "x": arr[:, 0], "y": arr[:, 1],
                "head": arr[:, 2], "v": arr[:, 3],
            }

        # ---- pre-opt controls -----------------------------------------
        pre_ctrl = pr.pre_opt_controls
        if is_success and len(pre_ctrl) > 0:
            ctrl_num = len(ControlVar.DESCRIPTOR.fields)
            arr = np.zeros((len(pre_ctrl), ctrl_num))
            for idx, c in enumerate(pre_ctrl):
                arr[idx] = [c.accelerate, c.steer_angle]
            self.pre_opt_controls_dict = {
                "t": np.arange(len(pre_ctrl)),
                "acc": arr[:, 0], "steer": arr[:, 1],
            }

    # ==================================================================
    #  Drawing methods (each saves a separate .png)
    # ==================================================================

    def draw_main(self):
        """Render the Park overview with the shared Urban/Park renderer."""
        from pathlib import Path
        from utils.visualization.adapters.park import from_park
        from utils.visualization.export import save_png

        if not self.plan_res.solve_success:
            print("[MatplotlibVis] Park solve failed; skipping trajectory overview.")
            return
        scene = from_park(self.plan_problem, self.plan_res)
        save_png(scene, Path(self.save_dir) / "main.png")

    def draw_esdf(self):
        """ESDF contour map + occupancy scatter."""
        if self.safe_dis_map is None:
            print("[MatplotlibVis] No ESDF data, skipping esdf figure.")
            return

        fig, ax = plt.subplots(figsize=FIGURE_SIZE_ESDF)
        ax.set_aspect("equal")
        ax.set_title("ESDF — Euclidean Signed Distance Field")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

        # Occupied cells as black scatter
        occ = self.occ_pts_2d[self.occ_mask]
        if len(occ) > 0:
            ax.scatter(occ[:, 0], occ[:, 1], **RENDER["occ_scatter"])

        # Contour of safe distance (use world coordinates from occ_pts_2d)
        xs_2d = self.occ_pts_2d[:, 0].reshape(self.safe_dis_map.shape)
        ys_2d = self.occ_pts_2d[:, 1].reshape(self.safe_dis_map.shape)
        z = self.safe_dis_map

        # Mask NaN
        valid = np.isfinite(z)
        if valid.any():
            levels = np.linspace(np.nanmin(z), np.nanmax(z), 20)
            cs = ax.contourf(xs_2d, ys_2d, z, levels=levels, cmap="turbo", extend="both")
            ax.contour(xs_2d, ys_2d, z, levels=levels, colors="black", linewidths=0.3)
            plt.colorbar(cs, ax=ax, label="safe distance (m)")

        fig.tight_layout()
        path = os.path.join(self.save_dir, "esdf.png")
        fig.savefig(path, dpi=SAVE_DPI)
        plt.close(fig)
        print(f"[MatplotlibVis] saved {path}")

    def draw_speed(self):
        """Speed subplot."""
        pre = self.pre_opt_traj_dict
        opt = self.init_traj_dict
        if not pre and not opt:
            print("[MatplotlibVis] No trajectory data, skipping speed figure.")
            return

        fig, ax = plt.subplots(figsize=FIGURE_SIZE_SMALL)
        ax.set_title("Speed")
        ax.set_xlabel("time step")
        ax.set_ylabel("speed (m/s)")

        all_v = []
        if pre:
            ax.plot(pre["t"], pre["v"], **RENDER["pre_opt_speed"])
            all_v.append(pre["v"])
        if opt:
            ax.plot(opt["t"], opt["v"], **RENDER["init_speed"])
            all_v.append(opt["v"])

        if all_v:
            concat = np.concatenate([np.asarray(a) for a in all_v])
            lo, hi = self._calc_axis_margin(concat)
            ax.set_ylim(lo, hi)

        ax.legend(loc="upper left")
        fig.tight_layout()
        path = os.path.join(self.save_dir, "speed.png")
        fig.savefig(path, dpi=SAVE_DPI)
        plt.close(fig)
        print(f"[MatplotlibVis] saved {path}")

    def draw_acceleration(self):
        """Acceleration subplot."""
        pre = self.pre_opt_controls_dict
        opt = self.init_controls_dict
        if not pre and not opt:
            print("[MatplotlibVis] No control data, skipping acceleration figure.")
            return

        fig, ax = plt.subplots(figsize=FIGURE_SIZE_SMALL)
        ax.set_title("Acceleration")
        ax.set_xlabel("time step")
        ax.set_ylabel("acceleration (m/s²)")

        all_acc = []
        if pre:
            ax.plot(pre["t"], pre["acc"], **RENDER["pre_opt_acc"])
            all_acc.append(pre["acc"])
        if opt:
            ax.plot(opt["t"], opt["acc"], **RENDER["init_acc"])
            all_acc.append(opt["acc"])

        if all_acc:
            concat = np.concatenate([np.asarray(a) for a in all_acc])
            lo, hi = self._calc_axis_margin(concat)
            ax.set_ylim(lo, hi)

        ax.legend(loc="upper left")
        fig.tight_layout()
        path = os.path.join(self.save_dir, "acceleration.png")
        fig.savefig(path, dpi=SAVE_DPI)
        plt.close(fig)
        print(f"[MatplotlibVis] saved {path}")

    def draw_steering(self):
        """Steering subplot."""
        pre = self.pre_opt_controls_dict
        opt = self.init_controls_dict
        if not pre and not opt:
            print("[MatplotlibVis] No control data, skipping steering figure.")
            return

        fig, ax = plt.subplots(figsize=FIGURE_SIZE_SMALL)
        ax.set_title("Steering")
        ax.set_xlabel("time step")
        ax.set_ylabel("steering angle (rad)")

        all_steer = []
        if pre:
            ax.plot(pre["t"], pre["steer"], **RENDER["pre_opt_steer"])
            all_steer.append(pre["steer"])
        if opt:
            ax.plot(opt["t"], opt["steer"], **RENDER["init_steer"])
            all_steer.append(opt["steer"])

        if all_steer:
            concat = np.concatenate([np.asarray(a) for a in all_steer])
            lo, hi = self._calc_axis_margin(concat)
            ax.set_ylim(lo, hi)

        ax.legend(loc="upper left")
        fig.tight_layout()
        path = os.path.join(self.save_dir, "steering.png")
        fig.savefig(path, dpi=SAVE_DPI)
        plt.close(fig)
        print(f"[MatplotlibVis] saved {path}")

    # ==================================================================
    #  Run all
    # ==================================================================
    def run(self):
        print(f"[MatplotlibVis] Rendering figures to {self.save_dir} ...")
        self.draw_main()
        self.draw_esdf()
        self.draw_speed()
        self.draw_acceleration()
        self.draw_steering()
        print("[MatplotlibVis] Done.")


# ---------------------------------------------------------------------------
# CLI entry point (compatible with vis_main.py style)
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser("MatplotlibSavedResultVisualization")
    parser.add_argument(
        "--res_save_path", "-s", type=str, required=True,
        help="Directory containing plan_problem.pb and plan_res.pb",
    )
    parser.add_argument(
        "--problem_pb", type=str, default="plan_problem.pb",
        help="Filename of saved PlanProblem protobuf",
    )
    parser.add_argument(
        "--res_pb", type=str, default="plan_res.pb",
        help="Filename of saved PlanRes protobuf",
    )
    parser.add_argument(
        "--save_dir", type=str, default=None,
        help="Output directory for images (default: same as --res_save_path)",
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

    if args.save_dir is None:
        # Default: save the images alongside the protobuf results in the same
        # scenario directory (solve_results/park/场景_<solve time>).
        save_dir = os.path.normpath(args.res_save_path)
    else:
        save_dir = args.save_dir

    vis = MatplotlibVis(plan_problem, plan_res, save_dir, args.debug)
    vis.run()


if __name__ == "__main__":
    main()
