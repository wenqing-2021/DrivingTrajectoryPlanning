import yaml
import numpy as np
from protobuf.cost_map_pb2 import Point, CostMap
from protobuf.problem_pb2 import PlanProblem, MapBound, Polygon, PlanRes
from protobuf.kinematic_model_pb2 import StateVar, VehicleParam
from protobuf.params_pb2 import SolverParams
from utils.load_case import Case
from utils.math_utils import norm_angle


def convert_yaml_to_protobuf(file, protobuf_class):
    # read the yaml file
    with open(file, "r") as f:
        params = yaml.safe_load(f)
    # convert to protobuf
    attr_list = dir(protobuf_class)
    for key in attr_list:
        if key in params:
            setattr(protobuf_class, key, params[key])

    return protobuf_class


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

    # load vehicle params
    vehicle_param = convert_yaml_to_protobuf(
        "/root/workspace/AutomatedPark/src/config/vehicle_cfg.yaml", VehicleParam()
    )
    plan_problem.vehicle_param.CopyFrom(vehicle_param)

    return plan_problem


def load_solver_params(file):
    solver_params = convert_yaml_to_protobuf(file, SolverParams())

    return solver_params


def convert_rear_to_mid(rear_pts, rear_overhang, vehicle_len, theta):
    rear_pts_arr = np.array(rear_pts)
    revert_angle = norm_angle(theta - np.pi)
    back_pts = rear_pts_arr + np.array(
        [rear_overhang * np.cos(revert_angle), rear_overhang * np.sin(revert_angle)]
    )
    half_vehicle_len = vehicle_len / 2
    mid_pts = back_pts + np.array(
        [half_vehicle_len * np.cos(theta), half_vehicle_len * np.sin(theta)]
    )

    return mid_pts
