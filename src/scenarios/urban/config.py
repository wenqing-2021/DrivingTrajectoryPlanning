"""Configuration loading for the CommonRoad urban simulation."""

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Type, TypeVar

import yaml

_ConfigType = TypeVar("_ConfigType")


@dataclass(frozen=True)
class UrbanSimulationConfig:
    """Parameters owned by the Python closed-loop simulation."""

    max_steps: int = 120
    target_speed: float = 7.0
    goal_tolerance: float = 2.5

    def __post_init__(self) -> None:
        """Validate simulation parameters."""
        if self.max_steps <= 0:
            raise ValueError("simulation.max_steps must be positive")
        if self.target_speed <= 0.0:
            raise ValueError("simulation.target_speed must be positive")
        if self.goal_tolerance <= 0.0:
            raise ValueError("simulation.goal_tolerance must be positive")


@dataclass(frozen=True)
class MppiPlannerConfig:
    """Parameters consumed by the C++ MPPI planner."""

    horizon: int = 40
    num_samples: int = 768
    num_iterations: int = 4
    temperature: float = 1.0
    wheelbase: float = 2.7
    min_steering: float = -0.55
    max_steering: float = 0.55
    min_acceleration: float = -4.0
    max_acceleration: float = 2.5
    min_velocity: float = 0.0
    max_velocity: float = 15.0
    steering_noise: float = 0.2
    acceleration_noise: float = 1.0
    noise_smoothing: float = 0.95
    position_weight: float = 3.0
    heading_weight: float = 1.0
    velocity_weight: float = 2.0
    control_weight: float = 0.08
    control_rate_weight: float = 8.0
    terminal_weight: float = 8.0
    obstacle_weight: float = 35.0
    obstacle_softening: float = 2.5
    collision_cost: float = 1.0e5
    vehicle_radius: float = 1.4
    road_left_limit: float = 5.0
    road_right_limit: float = -2.0
    road_boundary_weight: float = 50.0
    random_seed: int = 42

    def __post_init__(self) -> None:
        """Validate MPPI parameters before constructing the native planner."""
        if self.horizon <= 0 or self.num_samples <= 0 or self.num_iterations <= 0:
            raise ValueError(
                "planner horizon, samples, and iterations must be positive"
            )
        if self.temperature <= 0.0 or self.wheelbase <= 0.0:
            raise ValueError("planner temperature and wheelbase must be positive")
        if self.steering_noise <= 0.0 or self.acceleration_noise <= 0.0:
            raise ValueError("planner noise standard deviations must be positive")
        if not 0.0 <= self.noise_smoothing < 1.0:
            raise ValueError("planner.noise_smoothing must be in [0, 1)")
        if self.obstacle_softening <= 0.0:
            raise ValueError("planner.obstacle_softening must be positive")
        if self.road_boundary_weight < 0.0 or self.road_right_limit >= self.road_left_limit:
            raise ValueError("planner road boundary configuration is invalid")
        if self.min_steering >= self.max_steering:
            raise ValueError("planner steering bounds are invalid")
        if self.min_acceleration >= self.max_acceleration:
            raise ValueError("planner acceleration bounds are invalid")
        if self.min_velocity >= self.max_velocity:
            raise ValueError("planner velocity bounds are invalid")
        if self.random_seed < 0:
            raise ValueError("planner.random_seed must be non-negative")

        weights = (
            self.position_weight,
            self.heading_weight,
            self.velocity_weight,
            self.control_weight,
            self.control_rate_weight,
            self.terminal_weight,
            self.obstacle_weight,
            self.obstacle_softening,
            self.collision_cost,
            self.vehicle_radius,
            self.road_boundary_weight,
        )
        if any(value < 0.0 for value in weights):
            raise ValueError("planner costs and vehicle radius must be non-negative")


@dataclass(frozen=True)
class UrbanConfig:
    """Complete urban simulation configuration."""

    simulation: UrbanSimulationConfig
    planner: MppiPlannerConfig


def default_config_path() -> Path:
    """Return the bundled urban MPPI configuration path."""
    source_root = Path(__file__).resolve().parents[2]
    return source_root / "config/urban_mppi.yaml"


def _parse_section(
    config_type: Type[_ConfigType],
    raw_config: Dict[str, Any],
    section_name: str,
) -> _ConfigType:
    """Parse one YAML section and reject misspelled parameter names."""
    section = raw_config.get(section_name, {})
    if not isinstance(section, dict):
        raise ValueError(f"{section_name} must be a YAML mapping")

    valid_names = {field.name for field in fields(config_type)}
    unknown_names = set(section) - valid_names
    if unknown_names:
        unknown_list = ", ".join(sorted(unknown_names))
        raise ValueError(f"Unknown {section_name} parameters: {unknown_list}")
    return config_type(**section)


def load_config(config_path: Path) -> UrbanConfig:
    """Load and validate an urban MPPI YAML configuration."""
    with config_path.open("r", encoding="utf-8") as config_file:
        raw_config = yaml.safe_load(config_file) or {}
    if not isinstance(raw_config, dict):
        raise ValueError("Urban configuration must be a YAML mapping")

    valid_sections = {"simulation", "planner"}
    unknown_sections = set(raw_config) - valid_sections
    if unknown_sections:
        unknown_list = ", ".join(sorted(unknown_sections))
        raise ValueError(f"Unknown urban configuration sections: {unknown_list}")

    return UrbanConfig(
        simulation=_parse_section(UrbanSimulationConfig, raw_config, "simulation"),
        planner=_parse_section(MppiPlannerConfig, raw_config, "planner"),
    )
