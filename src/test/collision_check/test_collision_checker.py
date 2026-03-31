import sys
import os

# get the current project root directory
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

INSTALL_PATH = os.path.join(PROJECT_ROOT, "build/install")
PYBIND_PATH = os.path.join(PROJECT_ROOT, "build/pybind_modules")
PROTOBUF_PATH = os.path.join(PROJECT_ROOT, "build/install/protobuf")
sys.path.append(PYBIND_PATH)
sys.path.append(PROTOBUF_PATH)
sys.path.append(INSTALL_PATH)
import solver_pybind
from utils.plan_utils import convert_yaml_to_protobuf
from protobuf.kinematic_model_pb2 import VehicleParam
import matplotlib.pyplot as plt
import numpy as np
import os


class Vehicle:
    def __init__(self):
        self.lw = 2.5  # wheelbase
        self.lf = 1.0  # front hang length
        self.lr = 1.0  # rear hang length
        self.lb = 2.0  # width

    def create_polygon(self, x, y, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        points = np.array(
            [
                [-self.lr, -self.lb / 2, 1],
                [self.lf + self.lw, -self.lb / 2, 1],
                [self.lf + self.lw, self.lb / 2, 1],
                [-self.lr, self.lb / 2, 1],
                [-self.lr, -self.lb / 2, 1],
            ]
        ).dot(
            np.array(
                [[cos_theta, -sin_theta, x], [sin_theta, cos_theta, y], [0, 0, 1]]
            ).transpose()
        )
        return points[:, 0:2]


def createTest():
    polygon_pts = []
    polygon_pts.append([1.0, 0.0])
    polygon_pts.append([1.0, 2.0])
    polygon_pts.append([-3.0, 5.0])
    polygon_pts.append([-5.0, 0.0])
    polygon_pts.append([-3.0, -5.0])

    vehicle = Vehicle()
    vehicle_pose = [0.0, 0.0, 0.0]
    vehicle_xy = [vehicle_pose[0], vehicle_pose[1]]
    theta = vehicle_pose[2]

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for i in range(len(polygon_pts)):
        ax.plot(
            [polygon_pts[i][0], polygon_pts[(i + 1) % len(polygon_pts)][0]],
            [polygon_pts[i][1], polygon_pts[(i + 1) % len(polygon_pts)][1]],
            "r",
        )

    # plot vehicle
    vehicle_pts = vehicle.create_polygon(vehicle_xy[0], vehicle_xy[1], theta)
    ax.plot(vehicle_pts[:, 0], vehicle_pts[:, 1], "b")
    save_path = "./figures"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    save_name = os.path.join(save_path, "test_collision.png")
    plt.savefig(save_name, dpi=300)

    return polygon_pts, vehicle_pose


def checkCollision():
    vehicle_cfg_path = os.path.join(PROJECT_ROOT, "src/config/vehicle_cfg.yaml")
    vehicle_param = convert_yaml_to_protobuf(vehicle_cfg_path, VehicleParam())
    vehicle_param_str = vehicle_param.SerializeToString()
    polygon_pts, vehicle_pose = createTest()
    # create collision checker
    is_collision = solver_pybind.TestCollisionChecker(
        polygon_pts, vehicle_pose, vehicle_param_str
    )

    return is_collision


if __name__ == "__main__":
    is_collision = checkCollision()
    print(is_collision)
