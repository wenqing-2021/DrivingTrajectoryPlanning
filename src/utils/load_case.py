import csv
import numpy as np
import enum


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
