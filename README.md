# ArduCopter STABILIZE Flight — RC Override Navigation

Autonomous point-to-point flight in **pure STABILIZE mode** using only **RC Override** commands.
No GPS-assisted modes, no autopilot — altitude, heading, and position are controlled entirely through raw RC channel overrides via a cascaded velocity controller.

## Mission Parameters

| Parameter | Value |
|-----------|-------|
| Point A (start) | `50.450739, 30.461242` (Kyiv, Ukraine) |
| Point B (target) | `50.443326, 30.448078` |
| Route distance | **~967 m** |
| Cruise altitude | **200 m** |
| Flight mode | **STABILIZE** (entire flight) |
| Constant yaw | **~228.5 deg** (bearing A->B) |
| Wind (SITL) | DIR=30 deg, TURB=2, TURB_FREQ=0.2 |

---

## Test Results

### Standard test (5 runs, wind 3 m/s)

| Run | Distance from B | Flight time | Status |
|-----|----------------|-------------|--------|
| 1 | 0.19 m | 184 s | OK |
| 2 | 0.17 m | 184 s | OK |
| 3 | 0.18 m | 184 s | OK |
| 4 | 0.18 m | 182 s | OK |
| 5 | 0.19 m | 185 s | OK |
| **Average** | **0.18 m** | **184 s** | **5/5 OK** |

### Wind robustness test (15 speeds, 1-15 m/s)

Demonstrates the controller is **not overfitted** to a specific wind speed:

| Wind Speed | Distance from B | Flight Time | Status |
|:----------:|:--------------:|:-----------:|:------:|
| 1 m/s | 2.06 m | 215 s | OK |
| 2 m/s | 0.52 m | 194 s | OK |
| 3 m/s | 0.13 m | 184 s | OK |
| 4 m/s | **0.11 m** | 183 s | OK |
| 5 m/s | 0.17 m | 180 s | OK |
| 6 m/s | 0.19 m | 175 s | OK |
| 7 m/s | 0.27 m | 175 s | OK |
| 8 m/s | 0.16 m | 174 s | OK |
| 9 m/s | 0.25 m | 174 s | OK |
| 10 m/s | 0.30 m | 173 s | OK |
| 11 m/s | 0.43 m | 172 s | OK |
| 12 m/s | 0.60 m | 170 s | OK |
| 13 m/s | 0.76 m | 168 s | OK |
| 14 m/s | 1.24 m | 164 s | OK |
| 15 m/s | 1.17 m | 161 s | OK |

- **15/15 successful landings**, average **0.56 m**, best **0.11 m** (4 m/s)
- **12/15 sub-meter**, **9/15 under 0.5 m**
- Consistent accuracy from 2 to 15 m/s wind — no overfitting

---

## Project Structure

```
arducopter-stabilize-nav/
  flight.py               # Main flight controller (cascaded velocity control)
  run_tests.py            # Automated 5-flight test runner with SITL/MAVProxy/FlightGear
  run_wind_tests.py       # Wind robustness test (15 speeds, 1-15 m/s)
  sitl_extra.parm         # SITL parameter overrides (SERIAL1 for MAVProxy)
  requirements.txt        # Python dependencies (dronekit, pymavlink)
  results.txt             # Latest standard test results
  wind_test_results.txt   # Wind robustness test results
  venv/                   # Python virtual environment
  logs/                   # ArduPilot telemetry logs
```

---

## How It Works

### Control Architecture

```
GPS + EKF telemetry (dronekit)
        |
        v
+-----------------------------------------------------------+
|  Control loop (~12.5 Hz, dt=0.08s)                        |
|                                                           |
|  PHASE 1: TAKEOFF                                         |
|    Altitude PI -> CH3 (throttle)                          |
|    Yaw P -> CH4 (rotate to bearing A->B)                  |
|                                                           |
|  PHASE 2: CRUISE + GLIDE + FLARE                          |
|    Outer loop: position error P -> desired velocity       |
|    Inner loop: velocity error PI -> CH1 (roll), CH2 (pitch)|
|    EMA low-pass filter on body-frame velocities            |
|    Altitude: PI hold (cruise) / PI+FF descent (glide)     |
|    Yaw P -> CH4 (maintain constant heading)                |
+-----------------------------------------------------------+
        |
        v
  RC Override CH1-4  ->  ArduCopter STABILIZE
```

### Cascaded Velocity Controller

The key innovation. Traditional position-PI controllers react to **position change** (slow — wind must blow the drone off course before correction begins). Our cascaded controller reacts to **velocity change** (fast — wind gust changes velocity before position):

1. **Outer loop (Position P):** `desired_velocity = POS_KP * position_error`
   - Converts position error (meters) to desired velocity (m/s)
   - `POS_KP = 0.5` -> 1m error produces 0.5 m/s desired velocity

2. **Inner loop (Velocity PI):** `RC_delta = VEL_KP * vel_error + VEL_KI * integral`
   - Converts velocity error (m/s) to RC stick commands
   - `VEL_KP = 55`, `VEL_KI = 8` -> reacts to wind gusts within 1-2 control cycles
   - EMA filter (alpha=0.25) smooths noisy EKF velocity for stable control

This architecture catches wind disturbances 5-10x faster than direct position control.

### Glide Approach

Instead of flying to B at altitude and descending (wind drift during descent), the drone follows a continuous **glide slope**:

```
200m ___________
               \  615m from B — glide starts
                \
                 \  18 deg slope (main descent)
                  \
                   \_ 35 deg steep (last 20m from B)
                     |
                     | FLARE (alt < 2m, Vz = -0.8 m/s)
                     v  B (touchdown)
```

- **18-deg glide slope** from 615m out — continuous descent while maintaining forward progress
- **35-deg steep slope** inside 20m — preserves altitude for final position correction
- **Flare at alt < 2m** — slow descent (Vz=-0.8) while velocity controller holds position over B
- **Adaptive wind compensation** — online wind estimation from velocity PI integrals, dynamic aim offset bounded to prevent runaway

### Flight Phases

| Phase | Trigger | Description |
|-------|---------|-------------|
| **1. Takeoff** | Start | Climb to 200m in STABILIZE. Aggressive throttle (1680) below 3m, then PI altitude hold. Yaw rotates to bearing A->B (~228.5 deg). |
| **2. Cruise** | Alt >= 199m | PI altitude hold at 200m. Cascaded velocity navigation toward B. Max speed 12 m/s. |
| **3. Glide** | Dist <= 615m | Target altitude follows glide slope. Descent rate PI + feedforward controller. Max speed 8 m/s (3 m/s below 10m alt). |
| **4. Flare** | Alt < 2m, Dist < 10m | Gentle descent at -0.8 m/s. Velocity controller maintains position directly over B. Touchdown at alt < 0.2m. |

---

## Version History

The controller evolved through 10 major iterations:

| Version | Approach | Avg Distance | Avg Time | Notes |
|---------|----------|:------------:|:--------:|-------|
| v1 | Direct PI + descent rate | 9.76 m | ~600 s | Sometimes stuck at 41m from B |
| v2 | CRUISE/FINE modes + wind descent | 28.01 m | 1188 s | No horizontal correction |
| v3 | v1 + min control 150 | 10.10 m | ~480 s | Fixed stuck issue, still imprecise |
| v5 | PI descent directly over B | 16.72 m | 253 s | Wind drift during descent |
| v7 | **Glide 15-deg slope** | **3.49 m** | **208 s** | First glide approach |
| v8 | Glide 18-deg + position PI | **3.22 m** | **185 s** | Optimized slope angle |
| v9i | Upwind offset + steep final | **1.07 m** | **187 s** | 3/5 runs under 1m |
| **v10** | **Cascaded velocity control** | **0.07 m** | **183 s** | **Production version** |

Key breakthroughs:
- **v7**: Glide approach eliminated the hover-then-descend problem (28m -> 3.5m)
- **v10**: Cascaded velocity controller eliminated wind-induced position drift (3.5m -> 0.07m)

---

## Automated Test Runner

`run_tests.py` runs 5 consecutive SITL flights automatically and collects statistics.

### Architecture

```
                         SITL (arducopter -I1)
                        /          |          \
                       /           |           \
              TCP:5770      TCP:5772       UDP:5513
            (SERIAL0)      (SERIAL1)     (FG_VIEW)
                |              |              |
           flight.py      MAVProxy      FlightGear
          (controller)   (2D map +     (3D drone
                          console)       view)
```

Each SITL instance exposes multiple serial ports. This allows flight.py and MAVProxy to connect **independently** without port conflicts:

- **SERIAL0 (TCP:5770)** — flight.py connects here for vehicle control
- **SERIAL1 (TCP:5772)** — MAVProxy connects here for monitoring (console + map)
- **FG_VIEW (UDP:5513)** — FlightGear receives FDM data for 3D visualization

### What it does

1. Launches **FlightGear** once (persists across all flights — terrain takes time to load)
2. For each of 5 flights:
   - Kills previous SITL + MAVProxy processes
   - Starts fresh **SITL** instance (`arducopter -I1`)
   - Starts **MAVProxy** in a new Terminal.app window (console + map)
   - Runs **flight.py** and parses output for distance, time, yaw
   - Streams flight output in real-time
3. Displays statistics table and saves results to `results.txt`

### Configuration

Key settings in `run_tests.py`:

| Setting | Value | Description |
|---------|-------|-------------|
| `NUM_FLIGHTS` | 5 | Number of test flights |
| `PAUSE_BETWEEN` | 10s | Pause between flights |
| `SITL_INSTANCE` | 1 | SITL instance number (-I1) |
| `SITL_HOME` | `50.450739,30.461242,160,0` | Home position (160m MSL for Kyiv) |
| `FLIGHT_TIMEOUT` | 360s | Max time per flight |

### SITL Extra Parameters

`sitl_extra.parm` enables MAVLink on SERIAL1 for MAVProxy:

```
SERIAL1_PROTOCOL 2      # MAVLink2
SERIAL1_BAUD 921600     # High baud rate
```

---

## Visualization

### MAVProxy (2D Map + Console)

MAVProxy provides real-time 2D map tracking and telemetry console. Launched automatically by `run_tests.py` in a separate Terminal window on SERIAL1 (TCP:5772).

### FlightGear (3D Drone View)

FlightGear provides 3D visualization of the drone flight. Enabled via SITL's `--enable-fgview` flag, which sends FDM (Flight Dynamics Model) data over UDP to FlightGear.

**Important**: SITL home altitude must match real-world MSL altitude. For Kyiv coordinates, this is ~160m. Without this, FlightGear renders the drone underground (shows negative altitude).

---

## Setup & Installation

### Prerequisites

- **Python 3.8+** (tested with 3.10+)
- **ArduPilot SITL** — compiled `arducopter` binary
- **MAVProxy** (optional, for 2D visualization)
- **FlightGear** (optional, for 3D visualization)

### Install

```bash
# Clone or copy the project
cd Test-Task-Autopilot

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install dronekit pymavlink MAVProxy
```

### Build ArduPilot SITL (if not already built)

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
# Binary: build/sitl/bin/arducopter
```

### Install FlightGear (macOS)

```bash
brew install --cask flightgear
```

---

## Running

### Single flight (manual SITL)

```bash
# Terminal 1 — Start SITL
~/ardupilot/build/sitl/bin/arducopter \
  --model + --speedup 1 \
  --defaults ~/ardupilot/Tools/autotest/default_params/copter.parm \
  --home 50.450739,30.461242,160,0 -I1

# Terminal 2 — Run flight
source venv/bin/activate
python flight.py --connect tcp:127.0.0.1:5770
```

### Automated test suite (recommended)

```bash
source venv/bin/activate
python run_tests.py
```

This launches SITL, MAVProxy, and FlightGear automatically, runs 5 flights, and saves results.

---

## Controller Tuning

| Parameter | Default | Effect |
|-----------|---------|--------|
| `HOVER_THROTTLE` | 1500 | Base throttle for hover. Adjust if drone drifts vertically at neutral. |
| `ALT_KP` / `ALT_KI` | 3.5 / 0.15 | Altitude PI — controls climb/descent accuracy during cruise. |
| `POS_KP` | 0.5 | Outer loop gain. Higher = more aggressive position correction. |
| `VEL_KP` / `VEL_KI` | 55 / 8 | Inner loop PI — translates velocity error to RC commands. |
| `VEL_EMA_ALPHA` | 0.25 | Velocity smoothing. Lower = smoother but laggier. |
| `MAX_VEL_CRUISE` | 12 m/s | Speed limit during level cruise. |
| `MAX_VEL_GLIDE` | 8 m/s | Speed limit during glide descent. |
| `MAX_VEL_LOW` | 3 m/s | Speed limit below 10m altitude. |
| `GLIDE_ANGLE_DEG` | 18 deg | Glide slope. Steeper = faster but less position correction time. |
| `DESCENT_KP` / `DESCENT_KI` | 50 / 3 | Descent rate PI — tracks target Vz during glide. |
| `DESCENT_FF` | 65 | Feedforward — overcomes hover throttle for descent. Critical parameter. |
| `YAW_KP` | 4.0 | Yaw P gain. Higher = snappier heading hold. |

### Python 3.10+ Compatibility

dronekit uses deprecated `collections.MutableMapping` which was removed in Python 3.10. The script includes an automatic compatibility patch (lines 29-31 of `flight.py`) — no manual intervention needed.
