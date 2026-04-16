"""Mission framework interfaces and shared data structures."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict

if TYPE_CHECKING:
    from .mission_context import MissionContext


class MissionStatus(Enum):
    """Status returned by a mission update step."""

    RUNNING = auto()
    WAITING = auto()
    SUCCESS = auto()
    FAILURE = auto()
    CANCELLED = auto()


class MissionFailurePolicy(Enum):
    """How the executor should react when a mission fails."""

    ABORT_AND_RTL = "abort_and_rtl"
    CONTINUE_TO_NEXT = "continue_to_next"

    @classmethod
    def from_value(cls, value: str | None) -> "MissionFailurePolicy":
        if value is None:
            return cls.ABORT_AND_RTL
        normalized = value.strip().lower()
        for member in cls:
            if member.value == normalized:
                return member
        raise ValueError(f"Unsupported mission failure policy: {value}")


@dataclass(frozen=True)
class MissionSpec:
    """Concrete mission instance configuration loaded from YAML."""

    type_name: str
    name: str
    config: Dict[str, Any]
    base_dir: Path
    failure_policy: MissionFailurePolicy = MissionFailurePolicy.ABORT_AND_RTL

    def resolve_path(self, raw_path: str) -> Path:
        candidate = Path(raw_path)
        if candidate.is_absolute():
            return candidate
        return (self.base_dir / candidate).resolve()


class BaseMission(ABC):
    """Base mission lifecycle interface."""

    def __init__(self, spec: MissionSpec) -> None:
        self.spec = spec
        self.name = spec.name

    @abstractmethod
    def on_enter(self, context: "MissionContext") -> None:
        """Prepare mission state before the first update tick."""

    @abstractmethod
    def update(self, context: "MissionContext") -> MissionStatus:
        """Advance mission state without blocking."""

    def on_exit(self, context: "MissionContext") -> None:
        """Cleanup hook after terminal completion."""

    def cancel(self, context: "MissionContext") -> None:
        """Called when the executor cancels the mission early."""
