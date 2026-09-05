"""
Main control loop.

Each tick:
  1. If we don't have a plan (or the world changed), run A* to get waypoints.
  2. Every `vision_check_interval` ticks, ask Gemini to look at the camera
     frame. If it flags something the planner didn't know about, we pause /
     turn instead of blindly following the A* waypoint.
  3. Otherwise, move toward the next waypoint.

This mirrors a common real-world pattern: a fast deterministic planner does
the routine work, and a slower "smarter" model acts as a semantic safety net.
"""
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from src.drone_interface import DroneInterface
from src.path_planner import a_star
from src.simulation_world import GridWorld
from src.vision import GeminiVision, VisionResult

Point = Tuple[int, int]


@dataclass
class NavigatorConfig:
    vision_check_interval: int = 5
    move_step_cm: int = 50
    waypoint_tolerance: float = 0.5
    max_ticks: int = 500


class Navigator:
    def __init__(
        self,
        drone: DroneInterface,
        world: GridWorld,
        vision: Optional[GeminiVision] = None,
        config: Optional[NavigatorConfig] = None,
    ):
        self.drone = drone
        self.world = world
        self.vision = vision
        self.config = config or NavigatorConfig()
        self.path: List[Point] = []
        self.path_index = 0
        self.tick = 0
        self.last_vision_result: Optional[VisionResult] = None

    def plan(self) -> None:
        state = self.drone.get_state()
        start = (int(round(state.x)), int(round(state.y)))
        path = a_star(self.world, start, self.world.goal)
        if path is None:
            raise RuntimeError("No path found to goal from current position.")
        self.path = path
        self.path_index = 0

    def _face_toward(self, target: Point) -> None:
        state = self.drone.get_state()
        dx = target[0] - state.x
        dy = target[1] - state.y
        target_heading = np.degrees(np.arctan2(dy, dx)) % 360
        turn = (target_heading - state.heading_deg + 180) % 360 - 180
        if abs(turn) > 1:
            self.drone.rotate(turn)

    def _distance_to(self, target: Point) -> float:
        state = self.drone.get_state()
        return ((state.x - target[0]) ** 2 + (state.y - target[1]) ** 2) ** 0.5

    def reached_goal(self) -> bool:
        return self._distance_to(self.world.goal) <= self.config.waypoint_tolerance

    def step(self) -> bool:
        """Run a single navigation tick. Returns True if the goal was reached."""
        if not self.path:
            self.plan()

        if self.reached_goal():
            return True

        if self.path_index >= len(self.path):
            self.plan()

        target = self.path[self.path_index]
        self._face_toward(target)

        vision_says_stop = False
        if self.vision and self.tick % self.config.vision_check_interval == 0:
            frame = self.drone.get_frame()
            result = self.vision.analyze_frame(frame)
            self.last_vision_result = result
            if not result.path_clear or result.suggested_action == "stop":
                print(f"[Navigator] Vision flagged: {result.obstacle_description!r} "
                      f"(action={result.suggested_action}, confidence={result.confidence:.2f})")
                vision_says_stop = True
            elif result.suggested_action in ("turn_left", "turn_right"):
                self.drone.rotate(15 if result.suggested_action == "turn_left" else -15)

        if not vision_says_stop:
            self.drone.move_forward(self.config.move_step_cm)
            if self._distance_to(target) <= self.config.waypoint_tolerance:
                self.path_index += 1

        self.tick += 1
        if self.tick > self.config.max_ticks:
            raise RuntimeError("Exceeded max navigation ticks without reaching goal.")
        return self.reached_goal()

    def run(self) -> None:
        self.drone.connect()
        self.drone.takeoff()
        try:
            while not self.step():
                pass
            print("[Navigator] Goal reached.")
        finally:
            self.drone.land()
