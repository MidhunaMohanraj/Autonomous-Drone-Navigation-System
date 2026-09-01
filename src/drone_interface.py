"""
Abstract interface every drone backend (simulated or real) must implement.
This is what lets Navigator run unchanged against either backend.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
import numpy as np


@dataclass
class DroneState:
    x: float
    y: float
    heading_deg: float  # 0 = facing +x (east), increases counter-clockwise
    battery_pct: int


class DroneInterface(ABC):
    """Common contract for SimulatedDrone and TelloDrone."""

    @abstractmethod
    def connect(self) -> None:
        ...

    @abstractmethod
    def takeoff(self) -> None:
        ...

    @abstractmethod
    def land(self) -> None:
        ...

    @abstractmethod
    def move_forward(self, distance_cm: int) -> None:
        ...

    @abstractmethod
    def rotate(self, degrees: int) -> None:
        """Positive = counter-clockwise (left turn), negative = clockwise."""
        ...

    @abstractmethod
    def get_state(self) -> DroneState:
        ...

    @abstractmethod
    def get_frame(self) -> np.ndarray:
        """Return current camera frame as an HxWx3 RGB numpy array."""
        ...

    @abstractmethod
    def emergency_stop(self) -> None:
        ...
