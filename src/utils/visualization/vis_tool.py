from bokeh.io import curdoc, output_notebook, push_notebook
from bokeh.layouts import column, row
from bokeh.models import (
    TextInput,
    Select,
    Button,
    Div,
    Spacer,
    Legend,
    Slider,
    ColumnDataSource,
    TabPanel,
    Tabs,
    Range1d,
    LinearAxis,
)
from bokeh.plotting import figure, show
from utils.visualization.vis_config import FIG_VIS, COLOR_MAP, RENDER_VIS
from utils.plan_utils import convert_rear_to_mid
from protobuf.problem_pb2 import PlanProblem, PlanRes
from protobuf.params_pb2 import SolverParams
from protobuf.cost_map_pb2 import CostMap, Point
from protobuf.kinematic_model_pb2 import StateVar, ControlVar
from typing import List
import numpy as np

# output_notebook()


class BokehVis:
    def __init__(
        self,
        plan_problem: PlanProblem,
        solver_params: SolverParams,
        plan_res: PlanRes,
        debug: bool = False,
    ):
        self.main_plotter = figure(**FIG_VIS["main_figure"])
        self.esdf_plotter = figure(**FIG_VIS["esdf_figure"])
        self.velocity_plotter = figure(**FIG_VIS["velocity_figure"])
        self.control_plotter = figure(**FIG_VIS["control_figure"])
        self.plan_problem = plan_problem
        self.solver_params = solver_params
        self.plan_res = plan_res
        self.init_traj_dict = {}
        self.init_controls_dict = {}
        self.doc = curdoc()
        self.debug = debug
        self._velocity_right_axis_added = False

        # build the tabs
        self._build_tabs()

        # build the plan problem
        self._build_plan_problem()

        # parse the plan result
        self._parse_plan_result()

    @staticmethod
    def _calc_axis_bounds(values: np.ndarray, padding_ratio: float = 0.1):
        values = np.asarray(values)
        if values.size == 0:
            return -1.0, 1.0

        finite_values = values[np.isfinite(values)]
        if finite_values.size == 0:
            return -1.0, 1.0

        min_v = float(np.min(finite_values))
        max_v = float(np.max(finite_values))
        span = max_v - min_v
        padding = max(span * padding_ratio, 1e-3)
        if span < 1e-9:
            padding = max(abs(max_v) * padding_ratio, 1e-2)

        return min_v - padding, max_v + padding

    def _build_tabs(self):
        layout_dict = self._build_layout()
        tab_list = []
        for key, value in layout_dict.items():
            tab = TabPanel(child=value, title=key)
            tab_list.append(tab)

        self.tabs = Tabs(tabs=tab_list)

    def _build_layout(self):
        layout_dict = {}
        main_layout = column(
            row(
                Spacer(width=FIG_VIS["row_margin_width"]),
                self.main_plotter,
                Spacer(width=FIG_VIS["row_margin_width"]),
                column(
                    row(
                        self.velocity_plotter,
                    ),
                    row(
                        self.control_plotter,
                    ),
                ),
            ),
        )
        esdf_layout = column(
            row(Spacer(width=FIG_VIS["row_margin_width"]), self.esdf_plotter),
        )

        layout_dict.update({"main_figure": main_layout, "esdf_figure": esdf_layout})

        return layout_dict

    def _build_plan_problem(self):
        # build the obstacle polygons
        obs_polygons_x_list = []
        obs_polygons_y_list = []
        for i in range(self.plan_problem.obstacle_num):
            obs_polygon = self.plan_problem.obstacle_list[i]
            obs_vertex_x_list = []
            obs_vertex_y_list = []
            for j in range(obs_polygon.vertex_num):
                obs_vertex_x_list.append(obs_polygon.vertex_pts[j].x)
                obs_vertex_y_list.append(obs_polygon.vertex_pts[j].y)

            obs_polygons_x_list.append(obs_vertex_x_list)
            obs_polygons_y_list.append(obs_vertex_y_list)

        obs_polygon_data = {
            "xs": obs_polygons_x_list,
            "ys": obs_polygons_y_list,
        }
        self.update_attr(obs_polygon_data, "obs_polygon")

        # build the goal and initial state
        # convert the rearer center to the mid point
        init_mid_pts = convert_rear_to_mid(
            [self.plan_problem.init_state.x, self.plan_problem.init_state.y],
            self.plan_problem.vehicle_param.rear_overhang,
            self.plan_problem.vehicle_param.length,
            self.plan_problem.init_state.theta,
        )
        initial_state_data = {
            "x": [init_mid_pts[0]],
            "y": [init_mid_pts[1]],
            "head": [self.plan_problem.init_state.theta],
            "vehicle_len": [self.plan_problem.vehicle_param.length],
            "vehicle_width": [self.plan_problem.vehicle_param.width],
        }
        self.update_attr(initial_state_data, "initial_state")
        goal_init_pts = convert_rear_to_mid(
            [self.plan_problem.goal_state.x, self.plan_problem.goal_state.y],
            self.plan_problem.vehicle_param.rear_overhang,
            self.plan_problem.vehicle_param.length,
            self.plan_problem.goal_state.theta,
        )
        goal_state_data = {
            "x": [goal_init_pts[0]],
            "y": [goal_init_pts[1]],
            "head": [self.plan_problem.goal_state.theta],
            "vehicle_len": [self.plan_problem.vehicle_param.length],
            "vehicle_width": [self.plan_problem.vehicle_param.width],
        }
        self.update_attr(goal_state_data, "goal_state")

    def _parse_plan_result(self):
        # parse the plan result
        # 1. parse esdf
        cost_map: CostMap = self.plan_res.cost_map
        safe_dis_map = np.zeros((cost_map.map_info.x_size, cost_map.map_info.y_size))
        occ_pts_map = np.zeros((cost_map.map_info.x_size, cost_map.map_info.y_size, 3))
        for point in cost_map.points:
            idx_x = point.index.idx_x
            idx_y = point.index.idx_y
            safe_dis_map[idx_x, idx_y] = point.safe_dis
            occ_pts_map[idx_x, idx_y] = [
                point.position.x,
                point.position.y,
                point.is_occupy,
            ]
        occ_flat_pts_map = occ_pts_map.reshape(-1, 3)
        safe_dis_map_src = dict(
            xs=occ_pts_map[..., 0],
            ys=occ_pts_map[..., 1],
            safe_dis=safe_dis_map,
        )
        occ_pts_map_src = dict(
            xs=occ_flat_pts_map[:, 0].tolist(),
            ys=occ_flat_pts_map[:, 1].tolist(),
            is_occ=occ_flat_pts_map[:, 2].tolist(),
        )
        self.update_attr(safe_dis_map_src, "safe_dis_map")
        self.update_attr(occ_pts_map_src, "occ_pts_map")

        # 2. parse init path
        is_success = self.plan_res.solve_success
        init_traj = self.plan_res.init_traj
        state_num = len(StateVar.DESCRIPTOR.fields)
        init_traj_stamp = np.arange(len(init_traj))
        # print("init_path size: ", len(init_path))
        init_traj_parse = np.zeros((len(init_traj), state_num))
        for idx, state in enumerate(init_traj):
            init_traj_parse[idx] = [state.x, state.y, state.theta, state.v]
            # print(f"state {idx}: {state.x}, {state.y}, {state.theta} \n")

        self.init_traj_dict = {
            "t": init_traj_stamp,
            "x": init_traj_parse[:, 0],
            "y": init_traj_parse[:, 1],
            "head": init_traj_parse[:, 2],
            "v": init_traj_parse[:, 3],
        }
        self.update_attr(self.init_traj_dict, "init_traj")

        # 3. parse init controls
        if is_success:
            init_controls = self.plan_res.init_controls
            control_num = len(ControlVar.DESCRIPTOR.fields)
            init_traj_stamp = np.arange(len(init_controls))
            init_controls_parse = np.zeros((len(init_controls), control_num))
            for idx, control in enumerate(init_controls):
                init_controls_parse[idx] = [control.accelerate, control.steer_angle]

            self.init_controls_dict = {
                "t": init_traj_stamp,
                "acc": init_controls_parse[:, 0],
                "steer": init_controls_parse[:, 1],
            }
            self.update_attr(self.init_controls_dict, "init_controls")

    def _render_plan_problem(self):
        # render the obstacles
        self.main_plotter.patches(
            xs="xs", ys="ys", source=self.obs_polygon, **RENDER_VIS["obs_polygon"]
        )
        self.main_plotter.rect(
            x="x",
            y="y",
            angle="head",
            width="vehicle_len",
            height="vehicle_width",
            source=self.initial_state,
            **RENDER_VIS["init_state"]
        )
        self.main_plotter.rect(
            x="x",
            y="y",
            angle="head",
            width="vehicle_len",
            height="vehicle_width",
            source=self.goal_state,
            **RENDER_VIS["goal_state"]
        )

    def render_esdf_map(self):
        self.esdf_plotter.contour(
            x=self.safe_dis_map.data["xs"],
            y=self.safe_dis_map.data["ys"],
            z=self.safe_dis_map.data["safe_dis"],
            levels=np.linspace(
                np.min(self.safe_dis_map.data["safe_dis"]),
                np.max(self.safe_dis_map.data["safe_dis"]),
                RENDER_VIS["esdf_contour"]["max_levels"],
            ),
            **RENDER_VIS["esdf_contour"]["contour"]
        )

    def _render_occ_map(self):
        self.esdf_plotter.scatter(
            x="xs",
            y="ys",
            alpha="is_occ",
            source=self.occ_pts_map,
            **RENDER_VIS["occ_square"]
        )

    def render_init_path(self):
        if hasattr(self, "init_traj"):
            self.main_plotter.line(
                x="x", y="y", source=self.init_traj, **RENDER_VIS["init_path"]
            )
            self.main_plotter.scatter(
                x="x", y="y", source=self.init_traj, **RENDER_VIS["init_path_scatter"]
            )

            # render rectangles
            init_path_x = self.init_traj_dict["x"]
            init_path_y = self.init_traj_dict["y"]
            init_path_head = self.init_traj_dict["head"]
            for idx in range(len(init_path_x)):
                # print(f"start to render rectangle{idx}")
                init_path_mid_pts = convert_rear_to_mid(
                    [init_path_x[idx], init_path_y[idx]],
                    self.plan_problem.vehicle_param.rear_overhang,
                    self.plan_problem.vehicle_param.length,
                    init_path_head[idx],
                )
                self.main_plotter.rect(
                    x=init_path_mid_pts[0],
                    y=init_path_mid_pts[1],
                    angle=init_path_head[idx],
                    width=self.plan_problem.vehicle_param.length,
                    height=self.plan_problem.vehicle_param.width,
                    **RENDER_VIS["init_path_rect"]
                )

    def render_init_traj(self):
        if hasattr(self, "init_traj"):
            speed_min, speed_max = self._calc_axis_bounds(self.init_traj_dict["v"])
            self.velocity_plotter.y_range = Range1d(start=speed_min, end=speed_max)

        if hasattr(self, "init_controls"):
            acc_min, acc_max = self._calc_axis_bounds(self.init_controls_dict["acc"])
            self.velocity_plotter.extra_y_ranges = {
                "acc": Range1d(start=acc_min, end=acc_max)
            }
            if not self._velocity_right_axis_added:
                self.velocity_plotter.add_layout(
                    LinearAxis(y_range_name="acc", axis_label="acceleration (m/s^2)"),
                    "right",
                )
                self._velocity_right_axis_added = True

        x_candidates = []
        if hasattr(self, "init_traj"):
            x_candidates.append(self.init_traj_dict["t"])
        if hasattr(self, "init_controls"):
            x_candidates.append(self.init_controls_dict["t"])
        if x_candidates:
            all_x = np.concatenate([np.asarray(x) for x in x_candidates])
            x_min, x_max = self._calc_axis_bounds(all_x, padding_ratio=0.03)
            self.velocity_plotter.x_range = Range1d(start=x_min, end=x_max)

        if hasattr(self, "init_traj"):
            self.velocity_plotter.line(
                x="t",
                y="v",
                source=self.init_traj,
                legend_label="speed (m/s)",
                **RENDER_VIS["init_traj"]
            )
            self.velocity_plotter.scatter(
                x="t", y="v", source=self.init_traj, **RENDER_VIS["init_traj_scatter"]
            )

        if hasattr(self, "init_controls"):
            self.velocity_plotter.line(
                x="t",
                y="acc",
                source=self.init_controls,
                y_range_name="acc",
                legend_label="acceleration (m/s^2)",
                **RENDER_VIS["init_control_acc"]
            )
            self.velocity_plotter.scatter(
                x="t",
                y="acc",
                source=self.init_controls,
                y_range_name="acc",
                **RENDER_VIS["init_control_acc_scatter"]
            )

        self.velocity_plotter.xaxis.axis_label = "time index"
        if self.velocity_plotter.left:
            self.velocity_plotter.left[0].axis_label = "speed (m/s)"
        if self.velocity_plotter.right:
            self.velocity_plotter.right[0].axis_label = "acceleration (m/s^2)"
        self.velocity_plotter.legend.location = "top_left"
        self.velocity_plotter.legend.click_policy = "hide"

    def render_init_controls(self):
        if hasattr(self, "init_controls"):
            x_min, x_max = self._calc_axis_bounds(
                self.init_controls_dict["t"], padding_ratio=0.03
            )
            steer_min, steer_max = self._calc_axis_bounds(
                self.init_controls_dict["steer"]
            )
            self.control_plotter.x_range = Range1d(start=x_min, end=x_max)
            self.control_plotter.y_range = Range1d(start=steer_min, end=steer_max)

        if hasattr(self, "init_controls"):
            self.control_plotter.line(
                x="t",
                y="steer",
                source=self.init_controls,
                legend_label="steer (rad)",
                **RENDER_VIS["init_control_steer"]
            )
            self.control_plotter.scatter(
                x="t",
                y="steer",
                source=self.init_controls,
                **RENDER_VIS["init_control_steer_scatter"]
            )

        self.control_plotter.xaxis.axis_label = "time index"
        self.control_plotter.yaxis.axis_label = "steering"
        self.control_plotter.legend.location = "top_left"
        self.control_plotter.legend.click_policy = "hide"

    def render(self):
        self._render_plan_problem()
        self._render_occ_map()
        self.render_esdf_map()
        self.render_init_path()
        self.render_init_traj()
        self.render_init_controls()

    def update_attr(self, data_dict: dict, attr_str: str):
        if not hasattr(self, attr_str):
            setattr(self, attr_str, ColumnDataSource(data=data_dict))
        else:
            getattr(self, attr_str).data = data_dict

    def run(
        self,
    ):
        # add the layout into the doc
        self.doc.add_root(self.tabs)
        self.doc.title = FIG_VIS["title"]
        self.render()
