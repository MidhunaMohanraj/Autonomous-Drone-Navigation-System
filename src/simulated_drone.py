"""
A drone backend that lives entirely in the GridWorld simulation.
No hardware, no network — good for fast iteration on navigation logic.
"""
import numpy as np

from src.drone_interface import DroneInterface, DroneState
from src.simulation_world import GridWorld

CELL_SIZE_CM = 50  # each grid cell represents 50cm, roughly matching Tello move increments


class SimulatedDrone(DroneInterface):
    def __init__(self, world: GridWorld):
        self.world = world
        self.x, self.y = world.start
        self.heading_deg = 0.0
        self.battery_pct = 100
        self._connected = False
        self._flying = False

    def connect(self) -> None:
        self._connected = True

    def takeoff(self) -> None:
        if not self._connected:
            raise RuntimeError("Call connect() before takeoff()")
        self._flying = True

    def land(self) -> None:
        self._flying = False

    def move_forward(self, distance_cm: int) -> None:
        if not self._flying:
            raise RuntimeError("Drone is not flying")
        cells = distance_cm / CELL_SIZE_CM
        rad = np.radians(self.heading_deg)
        new_x = self.x + np.cos(rad) * cells
        new_y = self.y + np.sin(rad) * cells

        if self.world.is_blocked(int(round(new_x)), int(round(new_y))):
            # Simulated collision avoidance: don't move into an obstacle.
            return
        self.x, self.y = new_x, new_y
        self.battery_pct = max(0, self.battery_pct - 1)

    def rotate(self, degrees: int) -> None:
        self.heading_deg = (self.heading_deg + degrees) % 360

    def get_state(self) -> DroneState:
        return DroneState(
            x=self.x, y=self.y, heading_deg=self.heading_deg, battery_pct=self.battery_pct
        )

    def get_frame(self) -> np.ndarray:
        return self.world.render_camera_frame(self.x, self.y, self.heading_deg)

    def emergency_stop(self) -> None:
        self._flying = False

    def at_goal(self, tolerance=0.6) -> bool:
        gx, gy = self.world.goal
        return ((self.x - gx) ** 2 + (self.y - gy) ** 2) ** 0.5 <= tolerance
