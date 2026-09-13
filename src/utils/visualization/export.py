"""PNG and GIF export with one shared renderer and explicit playback timing."""

from dataclasses import replace
from pathlib import Path

import numpy as np
from PIL import Image

from utils.visualization.model import Scene
from utils.visualization.renderer import RenderConfig, SceneRenderer


def save_png(
    scene: Scene,
    path: Path,
    *,
    time: float | None = None,
    config: RenderConfig | None = None,
) -> None:
    """Save an overview, or a snapshot at a supplied scene time."""
    if time is not None and (
        not np.isfinite(time) or not scene.times[0] <= time <= scene.times[-1]
    ):
        raise ValueError("Snapshot time must lie inside the recorded trajectory")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    renderer = SceneRenderer(scene, config)
    try:
        renderer.update(
            scene.times[-1] if time is None else time, overview=time is None
        )
        renderer.figure.savefig(path, format="png")
    finally:
        renderer.close()


DEFAULT_GIF_FPS = 10.0


def validate_gif_fps(value: float | str) -> float:
    """Validate the simulation sampling rate supported by GIF export."""
    fps = float(value)
    if not np.isfinite(fps) or not 10 <= fps <= 100:
        raise ValueError("GIF frequency must be between 10 and 100 Hz")
    return fps


def animation_times(
    scene: Scene, fps: float = DEFAULT_GIF_FPS, speed: float = 1.0
) -> tuple[np.ndarray, list[int]]:
    """Sample every 1/fps seconds, retaining any shorter final interval."""
    if scene.time_basis == "samples":
        raise ValueError("Missing timestamps: provide --dt for legacy Park GIF export")
    fps = validate_gif_fps(fps)
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError("Playback speed must be finite and positive")
    if fps * speed > 100:
        raise ValueError(
            "GIF playback exceeds 100 frames/s; reduce playback speed or FPS"
        )

    # Use a fixed simulation-time grid rather than linspace, which changes dt
    # whenever the trajectory duration is not an exact multiple of 1/fps.
    duration = float(scene.times[-1] - scene.times[0])
    count = int(np.floor(duration * fps))
    offsets = np.arange(count + 1, dtype=float) / fps
    if count > 0 and np.isclose(offsets[-1], duration, rtol=0, atol=1e-10):
        offsets[-1] = duration
    elif offsets[-1] < duration:
        offsets = np.append(offsets, duration)
    times = scene.times[0] + offsets
    times[-1] = scene.times[-1]

    # Speed only changes encoded delays, never the interpolation time grid.
    # Cumulative rounding avoids drift at rates such as 30 Hz. A sub-10 ms
    # terminal remainder still receives GIF's minimum positive delay.
    boundaries = np.rint(offsets * 100 / speed).astype(int)
    durations = (np.maximum(1, np.diff(boundaries)) * 10).tolist()
    durations.append(max(10, round(100 / (fps * speed)) * 10))
    return times, durations


def save_gif(
    scene: Scene,
    path: Path,
    *,
    fps: float = DEFAULT_GIF_FPS,
    speed: float = 1,
    loop: bool = True,
    config: RenderConfig | None = None,
) -> None:
    """Export a looping GIF with synchronized curves and both endpoint poses."""
    times, durations = animation_times(scene, fps, speed)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    renderer = SceneRenderer(scene, replace(config or RenderConfig(), curves=True))
    frames = []
    try:
        for time in times:
            renderer.update(float(time))
            renderer.figure.canvas.draw()
            rgba = np.asarray(renderer.figure.canvas.buffer_rgba())
            frames.append(
                Image.fromarray(rgba)
                .convert("RGB")
                .convert("P", palette=Image.Palette.ADAPTIVE)
            )
        options = {"loop": 0} if loop else {}
        frames[0].save(
            path,
            format="GIF",
            save_all=True,
            append_images=frames[1:],
            duration=durations,
            disposal=2,
            optimize=False,
            **options,
        )
    finally:
        renderer.close()
        for frame in frames:
            frame.close()
