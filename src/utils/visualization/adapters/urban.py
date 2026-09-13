"""Flatten CommonRoad geometry and occupancy into portable replay data."""

import numpy as np

from utils.visualization.model import DynamicFrame, PathLayer, Scene, Vehicle


def shape_polygons(shape) -> list[np.ndarray]:
    """Convert CommonRoad rectangles, circles, polygons, and shape groups."""
    if hasattr(shape, "shapes"):
        return [polygon for child in shape.shapes for polygon in shape_polygons(child)]
    if hasattr(shape, "vertices"):
        return [np.asarray(shape.vertices, dtype=float)]
    if hasattr(shape, "radius"):
        angles = np.linspace(0, 2 * np.pi, 49)[:-1]
        return [
            np.asarray(shape.center)
            + shape.radius * np.column_stack((np.cos(angles), np.sin(angles)))
        ]
    raise ValueError(f"Unsupported CommonRoad shape: {type(shape).__name__}")


def from_urban(urban, result, *, vehicle: Vehicle | None = None) -> Scene:
    """Capture map, route, goal regions and discrete obstacle occupancies."""
    scenario = urban.scenario
    initial_step = int(urban.planning_problem.initial_state.time_step)
    times = (initial_step + np.arange(len(result.states))) * scenario.dt
    layers = []
    for lane in scenario.lanelet_network.lanelets:
        layers.append(
            PathLayer(
                np.vstack((lane.left_vertices, lane.right_vertices[::-1])), kind="road"
            )
        )
        for boundary in (lane.left_vertices, lane.right_vertices):
            layers.append(PathLayer(boundary, kind="boundary"))
    for obstacle in scenario.static_obstacles:
        occupancy = obstacle.occupancy_at_time(initial_step)
        if occupancy is not None:
            layers.extend(
                PathLayer(p, kind="obstacle") for p in shape_polygons(occupancy.shape)
            )
    for goal in urban.planning_problem.goal.state_list:
        if hasattr(goal, "position"):
            layers.extend(
                PathLayer(p, "Goal region", "goal")
                for p in shape_polygons(goal.position)
            )
    layers.append(PathLayer(urban.reference_path, "Reference", "reference"))
    dynamic = []
    for index, time in enumerate(times):
        polygons = []
        for obstacle in scenario.dynamic_obstacles:
            occupancy = obstacle.occupancy_at_time(initial_step + index)
            if occupancy is not None:
                polygons.extend(shape_polygons(occupancy.shape))
        dynamic.append(DynamicFrame(float(time), polygons))
    # Urban's state is the planner reference position. Body size is a display
    # parameter; its collision model remains the configured circular envelope.
    return Scene(
        states=result.states,
        times=times,
        vehicle=vehicle or Vehicle(),
        controls=np.asarray(result.controls).reshape(-1, 2)[:, ::-1],
        layers=layers,
        dynamic=dynamic,
        start=result.states[0, :3].tolist(),
        title=f"Urban MPPI — goal reached: {result.reached_goal}",
        metadata={
            "scenario": "urban",
            "scenario_id": str(scenario.scenario_id),
            "dt": scenario.dt,
            "reached_goal": bool(result.reached_goal),
            "vehicle_geometry": "display geometry at planner reference position",
        },
    )
