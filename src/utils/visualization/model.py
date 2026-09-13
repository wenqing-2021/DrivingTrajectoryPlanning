"""Planner-independent scene data, geometry, and portable JSON replay files."""

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np


def norm_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def convert_rear_to_mid(rear_pts, rear_overhang, vehicle_len, theta):
    """Convert the rear-axle position using the existing Park geometry."""
    offset = vehicle_len / 2.0 - rear_overhang
    return np.asarray(rear_pts) + offset * np.array([np.cos(theta), np.sin(theta)])


def _array(value, columns: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.size == 0:
        result = np.empty((0, columns))
    if result.ndim != 2 or result.shape[1] != columns or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite [N, {columns}] array")
    return result


@dataclass
class Vehicle:
    """Vehicle dimensions in meters; offset is from pose to body center."""

    length: float = 4.5
    width: float = 2.0
    center_offset: float = 0.0
    wheel_base: float | None = None
    rear_overhang: float | None = None

    def __post_init__(self):
        """Validate geometry."""
        if not np.isfinite([self.length, self.width, self.center_offset]).all():
            raise ValueError("Vehicle dimensions must be finite")
        if self.length <= 0 or self.width <= 0:
            raise ValueError("Vehicle length and width must be positive")
        if self.wheel_base is not None and (
            not np.isfinite(self.wheel_base) or self.wheel_base <= 0
        ):
            raise ValueError("Wheelbase must be finite and positive")
        if self.rear_overhang is not None and (
            not np.isfinite(self.rear_overhang) or self.rear_overhang < 0
        ):
            raise ValueError("Rear overhang must be finite and nonnegative")

    def polygon(self, pose) -> np.ndarray:
        """Return the four body corners for [x, y, heading, ...]."""
        x, y, yaw = pose[:3]
        center = convert_rear_to_mid(
            [x, y], self.length / 2 - self.center_offset, self.length, yaw
        )
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
        corners = corners * [self.length / 2, self.width / 2]
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        return corners @ rotation.T + center

    def wheel_polygons(self, pose, steering: float = 0.0) -> list[np.ndarray]:
        """Rear wheels follow yaw; both front wheels use the bicycle steering angle."""
        x, y, yaw = pose[:3]
        wheel_base = (
            self.wheel_base if self.wheel_base is not None else 0.6 * self.length
        )
        rear_overhang = (
            self.rear_overhang
            if self.rear_overhang is not None
            else (self.length - wheel_base) / 2
        )
        rear_x = self.center_offset - self.length / 2 + rear_overhang
        rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        # Tire size and track width are display proportions, not collision geometry.
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])
        corners = corners * [self.length * 0.065, self.width * 0.065]
        wheels = []
        for axle_x, angle in ((rear_x, yaw), (rear_x + wheel_base, yaw + steering)):
            wheel_rotation = np.array(
                [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
            )
            for side in (-1, 1):
                center = np.array([axle_x, side * self.width * 0.46]) @ rotation.T + [
                    x,
                    y,
                ]
                wheels.append(corners @ wheel_rotation.T + center)
        return wheels


@dataclass
class PathLayer:
    """A named polyline or filled polygon in world coordinates."""

    points: np.ndarray
    name: str = ""
    kind: str = "reference"

    def __post_init__(self):
        """Validate layer coordinates and style role."""
        self.points = _array(self.points, 2, "Layer points")
        if self.kind not in {
            "road",
            "boundary",
            "obstacle",
            "goal",
            "reference",
            "initial",
        }:
            raise ValueError(f"Unknown layer kind: {self.kind}")
        minimum = 3 if self.kind in {"road", "obstacle", "goal"} else 2
        if len(self.points) < minimum:
            raise ValueError(f"{self.kind} requires at least {minimum} points")


@dataclass
class DynamicFrame:
    """Obstacle polygons at a scene sample; held until the next sample."""

    time: float
    polygons: list[np.ndarray]

    def __post_init__(self):
        """Validate occupancy snapshots."""
        if not np.isfinite(self.time):
            raise ValueError("Dynamic frame time must be finite")
        self.polygons = [_array(p, 2, "Obstacle") for p in self.polygons]
        if any(len(p) < 3 for p in self.polygons):
            raise ValueError("Obstacle polygons need at least three vertices")


@dataclass
class Scene:
    """Replay data with seconds, meters, radians and [acceleration, steering]."""

    states: np.ndarray  # [x, y, yaw, speed], speed is optional (three columns)
    times: np.ndarray
    vehicle: Vehicle = field(default_factory=Vehicle)
    controls: np.ndarray = field(default_factory=lambda: np.empty((0, 2)))
    layers: list[PathLayer] = field(default_factory=list)
    dynamic: list[DynamicFrame] = field(default_factory=list)
    start: list[float] | None = None
    goal: list[float] | None = None
    title: str = "Trajectory planning"
    time_basis: str = "recorded"  # recorded, assumed, or samples (static only)
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        """Reject inconsistent replay data before rendering."""
        self.states = np.asarray(self.states, dtype=float)
        if self.states.ndim != 2 or self.states.shape[1] not in (3, 4):
            raise ValueError("States must have shape [N, 3] or [N, 4]")
        if not len(self.states) or not np.isfinite(self.states).all():
            raise ValueError("States must be nonempty and finite")
        self.times = np.asarray(self.times, dtype=float)
        if self.times.shape != (len(self.states),) or not np.isfinite(self.times).all():
            raise ValueError("Each state needs one finite timestamp")
        if np.any(np.diff(self.times) <= 0):
            raise ValueError("State timestamps must be strictly increasing")
        self.controls = _array(self.controls, 2, "Controls")
        if len(self.controls) not in (0, len(self.states) - 1, len(self.states)):
            raise ValueError("Controls must have zero, N-1, or N rows for N states")
        if self.time_basis not in {"recorded", "assumed", "samples"}:
            raise ValueError("Unknown time basis")
        if any(
            b.time <= a.time
            for a, b in zip(self.dynamic, self.dynamic[1:], strict=False)
        ):
            raise ValueError("Dynamic frame times must be strictly increasing")
        for pose in (self.start, self.goal):
            if pose is not None and (len(pose) != 3 or not np.isfinite(pose).all()):
                raise ValueError("Start/goal poses must be finite [x, y, yaw]")

    def state_at(self, time: float) -> np.ndarray:
        """Interpolate position and unwrapped heading without changing speed sign."""
        values = self.states.copy()
        values[:, 2] = np.unwrap(values[:, 2])
        return np.array([np.interp(time, self.times, col) for col in values.T])

    def obstacles_at(self, time: float) -> list[np.ndarray]:
        """Return occupancy at the previous scene step, including empty frames."""
        index = np.searchsorted([f.time for f in self.dynamic], time, side="right") - 1
        return self.dynamic[index].polygons if index >= 0 else []

    def save(self, path: Path) -> None:
        """Save a self-contained, versioned replay without pickle or planner imports."""
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"version": 1, "scene": asdict(self)}
        path.write_text(
            json.dumps(payload, default=lambda a: a.tolist(), allow_nan=False),
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path: Path) -> "Scene":
        """Read and validate a portable replay."""
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("version") != 1:
            raise ValueError("Unsupported replay version")
        data = payload["scene"]
        data["vehicle"] = Vehicle(**data["vehicle"])
        data["layers"] = [PathLayer(**layer) for layer in data["layers"]]
        data["dynamic"] = [DynamicFrame(**frame) for frame in data["dynamic"]]
        return cls(**data)
