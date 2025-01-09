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
)
from bokeh.plotting import figure, show
from utils.visualization.vis_config import FIG_VIS, COLOR_MAP, RENDER_VIS
from utils.plan_utils import convert_rear_to_mid
from protobuf.problem_pb2 import PlanProblem, PlanRes
from protobuf.params_pb2 import SolverParams
from protobuf.cost_map_pb2 import CostMap, Point
from typing import List
import numpy as np

# output_notebook()


class BokehVis:
    def __init__(
        self, plan_problem: PlanProblem, solver_params: SolverParams, plan_res: PlanRes
    ):
        self.main_plotter = figure(**FIG_VIS["main_figure"])
        self.esdf_plotter = figure(**FIG_VIS["esdf_figure"])
        self.plan_problem = plan_problem
        self.solver_params = solver_params
        self.plan_res = plan_res
        self.doc = curdoc()

        # build the tabs
        self._build_tabs()

        # build the plan problem
        self._build_plan_problem()

        # parse the plan result
        self._parse_plan_result()

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
            row(Spacer(width=FIG_VIS["row_margin_width"]), self.main_plotter),
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

    def _render_esdf_map(self):
        pass

    def render(self):
        self._render_plan_problem()
        self._render_esdf_map()

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
        show(self.tabs)
