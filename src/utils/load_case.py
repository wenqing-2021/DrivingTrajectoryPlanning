import csv
import numpy as np
import enum
import yaml
from protobuf.cost_map_pb2 import Point, CostMap
from protobuf.problem_pb2 import PlanProblem, MapBound, Polygon, PlanRes
from protobuf.kinematic_model_pb2 import StateVar
from protobuf.params_pb2 import SolverParams


class CaseIndex(enum.Enum):
    X0 = 0
    Y0 = 1
    THETA0 = 2
    XF = 3
    YF = 4
    THETAF = 5
    OBS_NUM = 6
    OBS_START = 7
    nElement = 8


MAP_MARGIN = 12


class Case:
    def __init__(self):
        self.x0, self.y0, self.theta0 = 0, 0, 0
        self.xf, self.yf, self.thetaf = 0, 0, 0
        self.xmin, self.xmax = 0, 0
        self.ymin, self.ymax = 0, 0
        self.obs_num = 0
        self.obs = np.array([])

    def update(self, file):
        with open(file, "r") as f:
            reader = csv.reader(f)
            tmp = list(reader)
            v = [float(i) for i in tmp[0]]
            self.x0, self.y0, self.theta0 = v[
                CaseIndex.X0.value : CaseIndex.THETA0.value + 1
            ]
            self.xf, self.yf, self.thetaf = v[
                CaseIndex.XF.value : CaseIndex.THETAF.value + 1
            ]
            self.xmin = min(self.x0, self.xf) - MAP_MARGIN
            self.xmax = max(self.x0, self.xf) + MAP_MARGIN
            self.ymin = min(self.y0, self.yf) - MAP_MARGIN
            self.ymax = max(self.y0, self.yf) + MAP_MARGIN

            self.obs_num = int(v[CaseIndex.OBS_NUM.value])
            num_vertexes = np.array(
                v[CaseIndex.OBS_START.value : CaseIndex.OBS_START.value + self.obs_num],
                dtype=np.int32,
            )
            vertex_start = (
                CaseIndex.OBS_START.value
                + self.obs_num
                + (np.cumsum(num_vertexes, dtype=np.int32) - num_vertexes) * 2
            )
            self.obs = []
            for vs, nv in zip(vertex_start, num_vertexes):
                self.obs.append(
                    np.array(v[vs : vs + nv * 2]).reshape((nv, 2), order="A")
                )


def build_problem(file: str) -> PlanProblem:
    case = Case()
    case.update(file)
    print("Case loaded successfully!")
    plan_problem = PlanProblem()
    plan_problem.map_bound.CopyFrom(
        MapBound(max_x=case.xmax, min_x=case.xmin, max_y=case.ymax, min_y=case.ymin)
    )
    plan_problem.init_state.CopyFrom(StateVar(x=case.x0, y=case.y0, theta=case.theta0))
    plan_problem.goal_state.CopyFrom(StateVar(x=case.xf, y=case.yf, theta=case.thetaf))
    obstalce_num = case.obs_num
    plan_problem.obstacle_num = obstalce_num
    for i in range(obstalce_num):
        polygon = Polygon()
        polygon.vertex_num = len(case.obs[i])
        for j in range(len(case.obs[i])):
            point = Point()
            point.x = case.obs[i][j][0]
            point.y = case.obs[i][j][1]
            polygon.vertex_pts.append(point)

        plan_problem.obstacle_list.append(polygon)

    return plan_problem


def load_params(file):
    # read the yaml file
    with open(file, "r") as f:
        params = yaml.safe_load(f)
    # convert to protobuf
    solver_params = SolverParams()
    attr_list = dir(solver_params)
    for key in attr_list:
        if key in params:
            setattr(solver_params, key, params[key])

    return solver_params
