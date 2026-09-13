"""Shared headless Matplotlib rendering for Urban and Park scenes."""

from dataclasses import dataclass

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon

from utils.visualization.model import Scene
from utils.visualization.style import RENDER


@dataclass(frozen=True)
class RenderConfig:
    """Image geometry and optional synchronized control curves."""

    width: float = 12.0
    height: float = 6.0
    dpi: int = 120
    curves: bool = False

    def __post_init__(self):
        """Reject invalid image dimensions."""
        if not np.isfinite([self.width, self.height, self.dpi]).all():
            raise ValueError("Image dimensions and DPI must be finite")
        if min(self.width, self.height, self.dpi) <= 0:
            raise ValueError("Image dimensions and DPI must be positive")


class SceneRenderer:
    """Draw static scene layers once and update vehicle, trail and occupancy."""

    def __init__(self, scene: Scene, config: RenderConfig | None = None):
        """Build the figure using Park's existing color and vehicle conventions."""
        self.scene = scene
        config = config or RenderConfig()
        self.figure = plt.figure(figsize=(config.width, config.height), dpi=config.dpi)
        self.cursors = []
        channels = []
        if scene.states.shape[1] == 4:
            channels.append((scene.times, scene.states[:, 3], "Speed (m/s)"))
        if len(scene.controls):
            channels.extend(
                [
                    (
                        scene.times[: len(scene.controls)],
                        scene.controls[:, 0],
                        "Acc. (m/s²)",
                    ),
                    (
                        scene.times[: len(scene.controls)],
                        scene.controls[:, 1],
                        "Steer (rad)",
                    ),
                ]
            )
        if config.curves and channels:
            grid = self.figure.add_gridspec(len(channels), 3)
            self.ax = self.figure.add_subplot(grid[:, :2])
            for index, (times, values, label) in enumerate(channels):
                axis = self.figure.add_subplot(grid[index, 2])
                if label == "Speed (m/s)":
                    axis.plot(times, values, color="tab:blue")
                else:
                    if len(times) == len(scene.times) - 1:
                        times = np.append(times, scene.times[-1])
                        values = np.append(values, values[-1])
                    axis.step(times, values, where="post", color="tab:blue")
                axis.set_ylabel(label)
                axis.set_xlabel(
                    "Sample" if scene.time_basis == "samples" else "Time (s)"
                )
                axis.grid(alpha=0.2)
                self.cursors.append(axis.axvline(scene.times[0], color="tab:red"))
        else:
            self.ax = self.figure.add_subplot(111)
        ax = self.ax
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(alpha=0.15)
        for layer in scene.layers:
            points = layer.points
            if layer.kind in {"road", "obstacle", "goal"}:
                styles = {
                    "road": {"facecolor": "#f1f3f5", "edgecolor": "none", "zorder": 0},
                    "obstacle": RENDER["obs_polygon"],
                    "goal": {
                        "facecolor": "#dcf0df",
                        "edgecolor": "#448855",
                        "alpha": 0.6,
                    },
                }
                ax.add_patch(
                    Polygon(points, label=layer.name or None, **styles[layer.kind])
                )
            else:
                color = {
                    "boundary": "#999999",
                    "reference": "tab:blue",
                    "initial": RENDER["pre_opt_path"]["color"],
                }[layer.kind]
                ax.plot(
                    points[:, 0],
                    points[:, 1],
                    color=color,
                    linestyle="-" if layer.kind == "boundary" else "--",
                    linewidth=1.2,
                    label=layer.name or None,
                )
        for pose, name, style in (
            (scene.start, "Start", "init_state"),
            (scene.goal, "Goal", "goal_state"),
        ):
            if pose is not None:
                ax.add_patch(
                    Polygon(scene.vehicle.polygon(pose), label=name, **RENDER[style])
                )
        (self.trail,) = ax.plot(
            [],
            [],
            color=RENDER["init_path"]["color"],
            linewidth=2,
            label="Trajectory",
            zorder=5,
        )
        self.body = Polygon(
            scene.vehicle.polygon(scene.states[0]),
            facecolor="#ffdcad",
            edgecolor="#b86400",
            zorder=6,
        )
        ax.add_patch(self.body)
        self.wheels = PolyCollection(
            scene.vehicle.wheel_polygons(scene.states[0]),
            facecolors="#252525",
            edgecolors="black",
            linewidths=0.6,
            zorder=8,
        )
        ax.add_collection(self.wheels)
        (self.heading,) = ax.plot([], [], color="#7f4100", linewidth=2, zorder=7)
        self.obstacles = PolyCollection(
            [], facecolors="#808080", edgecolors="black", alpha=0.7, zorder=4
        )
        ax.add_collection(self.obstacles)
        self.status = ax.text(
            0.015,
            0.02,
            "",
            transform=ax.transAxes,
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.8, "edgecolor": "none"},
        )
        # Include complete body footprints and all moving obstacles for a fixed camera.
        points = [scene.vehicle.polygon(s) for s in scene.states]
        points.extend(layer.points for layer in scene.layers)
        points.extend(p for frame in scene.dynamic for p in frame.polygons)
        points.extend(
            scene.vehicle.polygon(p) for p in (scene.start, scene.goal) if p is not None
        )
        extent = np.vstack(points)
        lo, hi = extent.min(axis=0), extent.max(axis=0)
        margin = np.maximum((hi - lo) * 0.05, 1.0)
        ax.set_xlim(lo[0] - margin[0], hi[0] + margin[0])
        ax.set_ylim(lo[1] - margin[1], hi[1] + margin[1])
        handles, labels = ax.get_legend_handles_labels()
        unique = dict(zip(labels, handles, strict=True))
        ax.legend(unique.values(), unique.keys(), loc="upper left", fontsize=8)
        ax.set_title(scene.title)
        self.figure.tight_layout()

    def update(self, time: float, *, overview: bool = False) -> None:
        """Render one time using the same geometry for PNG snapshots and GIF frames."""
        scene = self.scene
        state = scene.state_at(time)
        polygon = scene.vehicle.polygon(state)
        self.body.set_xy(polygon)
        center = polygon.mean(axis=0)
        front = center + scene.vehicle.length * 0.45 * np.array(
            [np.cos(state[2]), np.sin(state[2])]
        )
        self.heading.set_data([center[0], front[0]], [center[1], front[1]])
        trail = (
            scene.states[:, :2]
            if overview
            else np.vstack((scene.states[scene.times <= time, :2], state[:2]))
        )
        self.trail.set_data(trail[:, 0], trail[:, 1])
        self.obstacles.set_verts(scene.obstacles_at(time))
        label = "sample" if scene.time_basis == "samples" else "t (s)"
        status = f"{label} = {time:.2f}"
        if scene.time_basis == "assumed":
            status += " (assumed timing)"
        if len(state) == 4:
            status += f"   v = {state[3]:.2f} m/s"
        # Controls apply on [t_i, t_{i+1}); there is no new command at terminal t_N.
        index = int(np.searchsorted(scene.times, time, side="right") - 1)
        # Hold the final wheel angle; absent controls mean straight wheels.
        steer = (
            scene.controls[np.clip(index, 0, len(scene.controls) - 1), 1]
            if len(scene.controls)
            else 0.0
        )
        self.wheels.set_verts(scene.vehicle.wheel_polygons(state, float(steer)))
        if 0 <= index < len(scene.controls):
            acc, steer = scene.controls[index]
            status += f"   a = {acc:.2f} m/s²   steer = {steer:.2f} rad"
        self.status.set_text(status)
        for cursor in self.cursors:
            cursor.set_xdata([time, time])

    def close(self) -> None:
        """Release the figure after export."""
        plt.close(self.figure)
