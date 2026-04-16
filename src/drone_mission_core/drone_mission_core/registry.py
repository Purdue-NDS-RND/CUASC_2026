"""Mission registry and YAML sequence loading."""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Dict, Iterable, List, Type

import yaml

from .mission_api import BaseMission, MissionFailurePolicy, MissionSpec


MISSION_REGISTRY: dict[str, Type[BaseMission]] = {}


def register_mission(type_name: str):
    """Decorator used by mission packages to register mission classes."""

    normalized = type_name.strip().lower()

    def decorator(cls: Type[BaseMission]) -> Type[BaseMission]:
        if normalized in MISSION_REGISTRY:
            raise ValueError(f"Mission type already registered: {normalized}")
        MISSION_REGISTRY[normalized] = cls
        return cls

    return decorator


def import_mission_modules(module_names: Iterable[str]) -> None:
    for module_name in module_names:
        if not module_name:
            continue
        importlib.import_module(module_name)


def create_mission(spec: MissionSpec) -> BaseMission:
    mission_cls = MISSION_REGISTRY.get(spec.type_name)
    if mission_cls is None:
        known = ", ".join(sorted(MISSION_REGISTRY.keys()))
        raise ValueError(
            f"Unknown mission type '{spec.type_name}'. Registered mission types: {known}"
        )
    return mission_cls(spec)


def load_sequence_file(sequence_file: str) -> List[MissionSpec]:
    path = Path(sequence_file).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"Mission sequence file not found: {path}")

    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    root = raw.get("mission_sequence", raw)
    raw_missions = root.get("missions")
    if not isinstance(raw_missions, list) or not raw_missions:
        raise ValueError(
            f"Mission sequence file '{path}' must contain a non-empty missions list"
        )

    default_policy = MissionFailurePolicy.from_value(root.get("on_failure"))
    specs: list[MissionSpec] = []
    for index, raw_mission in enumerate(raw_missions):
        if not isinstance(raw_mission, dict):
            raise ValueError(f"Mission entry #{index + 1} must be a mapping")

        type_name = raw_mission.get("type")
        if not isinstance(type_name, str) or not type_name.strip():
            raise ValueError(f"Mission entry #{index + 1} is missing a valid type")

        config = raw_mission.get("config", {})
        if config is None:
            config = {}
        if not isinstance(config, dict):
            raise ValueError(f"Mission entry #{index + 1} config must be a mapping")

        failure_policy = MissionFailurePolicy.from_value(
            raw_mission.get("failure_policy", default_policy.value)
        )

        name = raw_mission.get("name")
        if not isinstance(name, str) or not name.strip():
            name = f"{type_name.strip().lower()}_{index + 1}"

        specs.append(
            MissionSpec(
                type_name=type_name.strip().lower(),
                name=name.strip(),
                config=dict(config),
                base_dir=path.parent,
                failure_policy=failure_policy,
            )
        )
    return specs
