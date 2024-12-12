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
from protobuf.problem_pb2 import PlanProblem
from protobuf.params_pb2 import SolverParams

# output_notebook()


class BokehVis:
    def __init__(self, plan_problem: PlanProblem, solver_params: SolverParams):
        self.main_plotter = figure(**FIG_VIS["main_figure"])
        self.plan_problem = plan_problem
        self.solver_params = solver_params
        self.doc = curdoc()

        # build the tabs
        self._build_tabs()

        # build the plan problem
        self._build_plan_problem()

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

        layout_dict.update({"main_figure": main_layout})

        return layout_dict

    def _build_plan_problem(self):
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

    def _render_plan_problem(self):
        # render the obstacles
        self.main_plotter.patches(
            xs="xs", ys="ys", source=self.obs_polygon, **RENDER_VIS["obs_polygon"]
        )

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
        self._render_plan_problem()
        show(self.tabs)
