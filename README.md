# Autonomous Drone Navigation System

Autonomous drone navigation using **Gemini Vision** for semantic scene understanding,
combined with a classical **A\* path planner** for geometric obstacle avoidance.

Runs in two modes with the same code:

- 🖥️ **Simulation mode** — no hardware needed, runs entirely on your PC.
- 🚁 **Tello mode** — flies a real [DJI Tello](https://www.ryzerobotics.com/tello) (~$99, WiFi-controlled, official Python SDK).

Both modes implement the same `DroneInterface`, so the navigator logic doesn't change —
you literally swap one line to go from simulation to a real drone.

## Why Gemini + A\* (not just one or the other)

| | A\* path planner | Gemini Vision |
|---|---|---|
| Good at | Precise geometric routing on a known/measured map | Understanding *what* it's looking at ("that's a person", "glass door", "looks like fog ahead") |
| Bad at | Has no idea what an obstacle *is* | Too slow/rate-limited to call on every control tick |
| Role here | Runs every tick, cheap, deterministic | Runs every N ticks, semantic sanity check + dynamic obstacle flagging |

The `Navigator` merges both: A\* proposes the next waypoint, Gemini periodically inspects
the drone's camera frame and can veto/override ("stop, that's a person crossing").

## Architecture

```
src/
├── drone_interface.py   # Abstract base class both drones implement
├── simulated_drone.py   # Pure-Python simulated drone + camera renderer
├── tello_drone.py       # Real DJI Tello via djitellopy
├── simulation_world.py  # Grid world: obstacles, target, rendering
├── path_planner.py      # A* implementation
├── vision.py            # Gemini Vision wrapper (scene analysis -> structured JSON)
├── navigator.py         # Main control loop, merges planner + vision
└── config.py            # Env / API key loading

examples/
├── run_simulation.py    # Live pygame visualization, no hardware
└── run_tello.py         # Real-drone flight script

tests/
└── test_path_planner.py
```

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for details on the control loop.

## Setup

```bash
git clone <your-repo-url>
cd drone-autonomous-nav
python -m venv venv && source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -r requirements.txt
cp .env.example .env
```

Add your **free** Gemini API key to `.env` (get one at [aistudio.google.com/apikey](https://aistudio.google.com/apikey) — free tier is enough for this project):

```
GEMINI_API_KEY=your_key_here
```

## Run the simulation (no hardware required)

```bash
python examples/run_simulation.py
```

A pygame window opens showing a grid world, obstacles, the drone, and its planned path.
Every few steps the current camera frame is sent to Gemini, and its scene description
is printed to the console.

## Fly a real Tello

1. Power on the Tello and connect your computer to its WiFi network (`TELLO-XXXXXX`).
2. Make sure nothing else needs your normal WiFi/internet during flight (Tello mode is
   local-network only — Gemini calls will fail without internet, so the drone falls back
   to A*-only navigation if the API is unreachable — see `navigator.py`).
3. Fly in a large, open, obstacle-safe space. **Always be ready to catch/land it manually.**

```bash
python examples/run_tello.py
```

## Safety notes (read before flying a real drone)

- This is a hobbyist/educational project, not a certified flight system. Don't fly over
  people, roads, or private property, and check your local drone regulations.
- The Gemini-based obstacle checks are a *supplement* to, not a replacement for, the
  Tello's built-in sensors and your own line-of-sight control. Keep the remote/app handy
  to take over manually at any time.
- Battery cutoffs and emergency-land are handled in `tello_drone.py`, but always supervise
  real flights.

## Extending this project

- Swap the grid-world `A*` planner for RRT/RRT* if you want continuous-space planning.
- Add a real depth/obstacle sensor (Tello doesn't have one) via an add-on like the
  `Tello EDU` mission pad kit for more precise real-world positioning.
- Replace polling Gemini with a smaller local vision model for lower latency once you've
  validated the approach.

## License

MIT — see [`LICENSE`](LICENSE).
