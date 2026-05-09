from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class PedestrianPathProfile:
    pose: tuple[float, float, float, float, float, float] | None
    waypoints: list[tuple[float, float]] | None


def load_pedestrian_path_profile(
    config_path: Path | None,
    profile_name: str,
) -> PedestrianPathProfile | None:
    if config_path is None or not str(config_path).strip() or not profile_name.strip():
        return None
    if not config_path.is_file():
        raise FileNotFoundError(f'Pedestrian path config not found: {config_path}')

    with config_path.open('r', encoding='utf-8') as file_handle:
        data = yaml.safe_load(file_handle)

    if not isinstance(data, dict):
        raise ValueError('Pedestrian path config must be a mapping.')

    profiles = data.get('profiles', {})
    if not isinstance(profiles, dict):
        raise ValueError('Pedestrian path config field `profiles` must be a mapping.')

    raw_profile = profiles.get(profile_name)
    if raw_profile is None:
        available_profiles = ', '.join(sorted(str(key) for key in profiles))
        raise ValueError(
            f'Pedestrian path profile `{profile_name}` was not found in {config_path}. '
            f'Available profiles: {available_profiles or "<none>"}.'
        )
    if not isinstance(raw_profile, dict):
        raise ValueError(f'Pedestrian path profile `{profile_name}` must be a mapping.')

    pose = _parse_pose(raw_profile.get('pose'))
    waypoints = _parse_waypoints(raw_profile.get('waypoints'))
    return PedestrianPathProfile(pose=pose, waypoints=waypoints)


def _parse_pose(raw_pose: Any) -> tuple[float, float, float, float, float, float] | None:
    if raw_pose is None:
        return None
    if not isinstance(raw_pose, list) or len(raw_pose) < 6:
        raise ValueError('Pedestrian path profile `pose` must contain [x, y, z, roll, pitch, yaw].')
    return (
        float(raw_pose[0]),
        float(raw_pose[1]),
        float(raw_pose[2]),
        float(raw_pose[3]),
        float(raw_pose[4]),
        float(raw_pose[5]),
    )


def _parse_waypoints(raw_waypoints: Any) -> list[tuple[float, float]] | None:
    if raw_waypoints is None:
        return None
    if not isinstance(raw_waypoints, list):
        raise ValueError('Pedestrian path profile `waypoints` must be a list of [x, y] points.')

    waypoints: list[tuple[float, float]] = []
    for item in raw_waypoints:
        if not isinstance(item, list) or len(item) < 2:
            raise ValueError('Each pedestrian path profile waypoint must be [x, y].')
        waypoints.append((float(item[0]), float(item[1])))
    return waypoints
