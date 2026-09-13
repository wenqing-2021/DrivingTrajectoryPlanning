"""Behavior tests for shared geometry, portable replay, and export timing."""

import json

import numpy as np
import pytest
from PIL import Image, ImageSequence

from utils.visualization.export import animation_times, save_gif, save_png
from utils.visualization.model import DynamicFrame, PathLayer, Scene, Vehicle
from utils.visualization.renderer import RenderConfig, SceneRenderer


def sample_scene():
    return Scene(
        states=[
            [0, 0, np.deg2rad(179), -1],
            [1, 0, np.deg2rad(-179), -1],
            [4, 0, np.deg2rad(-175), 0],
        ],
        times=[0, 0.2, 1],
        controls=[[0, 0.1], [1, 0.2]],
        vehicle=Vehicle(4.5, 2, 1.25),
        layers=[PathLayer([[0, -3], [4, -3]], "Reference")],
        dynamic=[
            DynamicFrame(0, [np.array([[1, 2], [2, 2], [1, 3]])]),
            DynamicFrame(0.2, []),
        ],
    )


def test_heading_wrap_reverse_and_rear_axle_geometry():
    scene = sample_scene()
    midpoint = scene.state_at(0.1)
    np.testing.assert_allclose(midpoint, [0.5, 0, np.pi, -1])
    np.testing.assert_allclose(
        scene.vehicle.polygon([0, 0, 0]), [[-1, -1], [3.5, -1], [3.5, 1], [-1, 1]]
    )
    np.testing.assert_allclose(
        scene.vehicle.polygon([2, 3, np.pi / 2]).mean(axis=0), [2, 4.25]
    )
    assert len(scene.obstacles_at(0.199)) == 1
    assert scene.obstacles_at(0.2) == []
    assert scene.obstacles_at(-1) == []


def test_wheel_axles_and_steering_in_world_coordinates():
    vehicle = Vehicle(4.5, 2, 1.25, wheel_base=2.7, rear_overhang=1)
    wheels = vehicle.wheel_polygons([2, 3, np.pi / 2], steering=-0.3)
    centers = np.array([wheel.mean(axis=0) for wheel in wheels])
    np.testing.assert_allclose(centers[:2].mean(axis=0), [2, 3])
    np.testing.assert_allclose(centers[2:].mean(axis=0), [2, 5.7])
    for index, wheel in enumerate(wheels):
        direction = wheel[1] - wheel[0]
        angle = np.arctan2(direction[1], direction[0])
        assert angle == pytest.approx(np.pi / 2 - (0.3 if index >= 2 else 0))


def test_wheels_follow_control_intervals_and_hold_terminal_angle():
    scene = sample_scene()
    scene.controls[:, 1] = [-0.3, 0.4]
    renderer = SceneRenderer(scene)
    try:
        for time, steering in [(0, -0.3), (0.199, -0.3), (0.2, 0.4), (1, 0.4)]:
            renderer.update(time)
            wheels = renderer.wheels.get_paths()
            state = scene.state_at(time)
            for index, wheel in enumerate(wheels):
                edge = wheel.vertices[1] - wheel.vertices[0]
                expected = state[2] + (steering if index >= 2 else 0)
                np.testing.assert_allclose(
                    edge / np.linalg.norm(edge),
                    [np.cos(expected), np.sin(expected)],
                    atol=1e-12,
                )
    finally:
        renderer.close()


@pytest.mark.parametrize("scenario", ["park", "urban"])
def test_gif_defaults_to_curves_and_wheels(tmp_path, monkeypatch, scenario):
    from utils.visualization import export

    scene = sample_scene()
    scene.metadata["scenario"] = scenario
    renderers = []

    def capture_renderer(scene, config):
        renderer = SceneRenderer(scene, config)
        renderers.append(renderer)
        return renderer

    monkeypatch.setattr(export, "SceneRenderer", capture_renderer)
    save_gif(scene, tmp_path / f"{scenario}.gif", config=RenderConfig(8, 5, 40))
    assert len(renderers[0].figure.axes) == 4
    assert len(renderers[0].cursors) == 3
    assert len(renderers[0].wheels.get_paths()) == 4


def test_portable_replay_round_trip(tmp_path):
    scene = sample_scene()
    path = tmp_path / "replay.json"
    scene.save(path)
    restored = Scene.load(path)
    np.testing.assert_allclose(restored.state_at(0.6), scene.state_at(0.6))
    np.testing.assert_allclose(restored.controls, scene.controls)
    np.testing.assert_allclose(restored.dynamic[0].polygons, scene.dynamic[0].polygons)
    # Replays written before axle dimensions were added remain readable.
    payload = json.loads(path.read_text())
    payload["scene"]["vehicle"].pop("wheel_base")
    payload["scene"]["vehicle"].pop("rear_overhang")
    path.write_text(json.dumps(payload))
    assert len(Scene.load(path).vehicle.wheel_polygons([0, 0, 0])) == 4
    payload["version"] = 999
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match="version"):
        Scene.load(path)


@pytest.mark.parametrize(
    "states,times,controls",
    [
        ([], [], []),
        ([[0, 0, 0]], [float("nan")], []),
        ([[0, 0, 0], [1, 0, 0]], [0, 0], []),
        ([[0, 0, 0], [1, 0, 0]], [0, 1], [[0, 0]] * 3),
    ],
)
def test_invalid_trajectory_rejected(states, times, controls):
    with pytest.raises(ValueError):
        Scene(states=states, times=times, controls=controls)


def test_animation_timing_and_terminal_state():
    scene = sample_scene()
    times, durations = animation_times(scene, fps=15, speed=2)
    assert times[0] == 0 and times[-1] == 1
    assert abs(sum(durations[:-1]) - 500) <= 10
    assert all(d > 0 and d % 10 == 0 for d in durations)
    scene.time_basis = "samples"
    with pytest.raises(ValueError, match="Missing timestamps"):
        animation_times(scene, 15, 1)


@pytest.mark.parametrize(
    "fps,speed",
    [
        (0, 1),
        (5, 1),
        (9.99, 1),
        (101, 1),
        (15, 0),
        (float("nan"), 1),
        (100, 2),
    ],
)
def test_invalid_playback_rejected(fps, speed):
    with pytest.raises(ValueError):
        animation_times(sample_scene(), fps, speed)


def test_png_gif_and_synchronized_artists(tmp_path):
    scene = sample_scene()
    config = RenderConfig(8, 4, dpi=60, curves=True)
    renderer = SceneRenderer(scene, config)
    try:
        renderer.update(0)
        assert len(renderer.obstacles.get_paths()) == 1
        renderer.update(1)
        assert not renderer.obstacles.get_paths()
        np.testing.assert_allclose(
            renderer.body.get_xy()[:4], scene.vehicle.polygon(scene.states[-1])
        )
        assert (
            "a =" not in renderer.status.get_text()
        )  # N-1 controls: no terminal command
        assert all(list(cursor.get_xdata()) == [1, 1] for cursor in renderer.cursors)
    finally:
        renderer.close()
    save_png(scene, tmp_path / "overview.png", config=config)
    save_png(scene, tmp_path / "snapshot.png", time=0.1, config=config)
    with Image.open(tmp_path / "overview.png") as png:
        assert png.size == (480, 240)
    with pytest.raises(ValueError, match="Snapshot time"):
        save_png(scene, tmp_path / "bad.png", time=2)
    save_gif(scene, tmp_path / "trajectory.gif", config=config)
    with Image.open(tmp_path / "trajectory.gif") as gif:
        assert gif.n_frames == 11
        assert gif.info["loop"] == 0
        assert all(f.info["duration"] == 100 for f in ImageSequence.Iterator(gif))
        assert sum(f.info["duration"] for f in ImageSequence.Iterator(gif)) == 1100
        gif.seek(0)
        first = np.array(gif.convert("RGB"))
        gif.seek(gif.n_frames - 1)
        assert not np.array_equal(first, np.array(gif.convert("RGB")))


def test_single_frame_and_no_controls(tmp_path):
    scene = Scene(states=[[0, 0, 0]], times=[3])
    save_gif(scene, tmp_path / "one.gif", loop=False, config=RenderConfig(4, 3, 40))
    with Image.open(tmp_path / "one.gif") as gif:
        assert gif.n_frames == 1
        assert "loop" not in gif.info


def test_replay_cli_format_selection_and_snapshot(tmp_path):
    from utils.visualization.cli import main

    source = tmp_path / "input" / "replay.json"
    sample_scene().save(source)
    output = tmp_path / "snapshot"
    assert (
        main(
            [
                str(source),
                "--output",
                str(output),
                "--visualize",
                "png",
                "--time",
                "0.1",
                "--dpi",
                "40",
            ]
        )
        == 0
    )
    assert (output / "trajectory.png").is_file()
    assert not (output / "trajectory.gif").exists()
    assert Scene.load(output / "replay.json").time_basis == "recorded"
    with pytest.raises(SystemExit) as error:
        main([str(source), "--output", str(output), "--visualize", "gif", "--fps", "0"])
    assert error.value.code == 2


@pytest.mark.parametrize("fps", [10, 20, 30])
def test_fixed_sampling_of_nonuniform_xyv(fps):
    scene = Scene(
        states=[[0, 0, 0, -2], [2, 4, 0.2, 0], [6, 8, 0.4, 4]],
        times=[2, 2.17, 2.43],
    )
    times, durations = animation_times(scene, fps=fps)
    expected_grid = 2 + np.arange(int(0.43 * fps) + 1) / fps
    np.testing.assert_allclose(times[:-1], expected_grid)
    assert times[-1] == 2.43
    np.testing.assert_allclose(np.diff(times[:-1]), 1 / fps)
    assert 0 < times[-1] - times[-2] < 1 / fps
    states = np.array([scene.state_at(t) for t in times])
    for col in (0, 1, 3):
        np.testing.assert_allclose(
            states[:, col], np.interp(times, scene.times, scene.states[:, col])
        )
    np.testing.assert_allclose(states[-1], scene.states[-1])
    assert abs(sum(durations[:-1]) - 430) <= 10
    faster_times, faster_durations = animation_times(scene, fps=fps, speed=2)
    np.testing.assert_array_equal(faster_times, times)
    assert abs(sum(faster_durations[:-1]) - 215) <= 10


def test_default_sampling_and_interpolated_values():
    scene = Scene(
        states=[[0, 0, 0, -2], [2, 4, 0.2, 0], [6, 8, 0.4, 4]], times=[0, 0.2, 0.45]
    )
    times, durations = animation_times(scene)
    np.testing.assert_allclose(times, [0, 0.1, 0.2, 0.3, 0.4, 0.45])
    np.testing.assert_allclose(scene.state_at(times[1]), [1, 2, 0.1, -1])
    np.testing.assert_allclose(scene.state_at(times[3]), [3.6, 5.6, 0.28, 1.6])
    assert durations == [100, 100, 100, 100, 50, 100]


def test_shared_cli_default_and_minimum_frequency():
    import argparse

    from utils.visualization.cli import add_render_arguments

    parser = argparse.ArgumentParser()
    add_render_arguments(parser)
    assert parser.parse_args([]).fps == 10
    assert parser.parse_args(["--fps", "20"]).fps == 20
    with pytest.raises(SystemExit) as error:
        parser.parse_args(["--fps", "9.99"])
    assert error.value.code == 2
