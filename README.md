# ArduCopter STABILIZE Flight — RC Override Navigation (Glide Approach)

Autonomous A->B flight in **STABILIZE mode** using **RC Override**.
No assisted flight modes — altitude, heading and position are controlled entirely through raw RC channel overrides.

| Parameter | Value |
|-----------|-------|
| Point A (start) | `50.450739, 30.461242` |
| Point B (target) | `50.443326, 30.448078` |
| Cruise altitude | **200 m** |
| Flight mode | **STABILIZE** throughout |
| Constant Yaw | **~228.5 deg** (bearing A->B, fixed for the entire flight) |
| Wind (SITL) | SPD=3, DIR=30 deg, TURB=2, TURB\_FREQ=0.2 |

## Results (5 test runs)

| Run | Distance from B | Flight time |
|-----|----------------|-------------|
| 1 | 3.22 m | 185 s |
| 2 | 4.13 m | 184 s |
| 3 | 3.72 m | 184 s |
| 4 | 3.56 m | 185 s |
| 5 | 3.74 m | 185 s |
| **Average** | **3.67 m** | **184.6 s** |

---

## How it works

### Architecture

```
GPS telemetry (dronekit)
        |
        v
+-----------------------------------------------+
|  Control loop  (~12.5 Hz)                      |
|                                                |
|  Phase 1: TAKEOFF                              |
|    Altitude PI -> CH3 (throttle)               |
|    Yaw P -> CH4 (rotate to bearing A->B)       |
|                                                |
|  Phase 2: CRUISE + GLIDE (single loop)         |
|    if dist > glide_start: altitude hold at 200m|
|    if dist <= glide_start: follow glide slope   |
|      target_alt = dist * tan(18 deg)           |
|      descent rate PI + feedforward -> CH3      |
|    NED -> body frame PI -> CH1/CH2             |
|    Yaw P -> CH4                                |
+------------------------------------------------+
        |
        v
  RC Override CH1-4  ->  ArduCopter STABILIZE
```

### Glide approach (key innovation)

Instead of flying to B at altitude and then descending (which causes wind drift),
the drone follows an **18-degree glide slope**:

```
200m ___________
               \  glide_start = 615m from B
                \
                 \  target_alt = dist_to_B * tan(18 deg)
                  \
                   \
                    \____  B (ground)
```

- No separate landing phase — no hover, no momentum zeroing
- Drone continuously approaches B while descending
- Wind compensation: aim point offset upwind (~15% of raw drift)
- PI position correction throughout entire descent
- Boosted nav gains at low altitude for precision

### Flight phases

| Phase | Description |
|-------|-------------|
| **1. Takeoff** | STABILIZE mode, aggressive climb to 200m with PI altitude hold. Yaw rotates to bearing A->B. |
| **2. Cruise** | PI altitude hold at 200m, NED position error rotated to body frame -> CH1/CH2 with PI control. Yaw locked to 228.5 deg via CH4. |
| **2b. Glide** | When dist <= 615m, target altitude follows glide slope. Descent rate PI + feedforward controller. Full PI position correction toward B (with wind aim offset). |

### Key design decisions

**Glide approach instead of hover-then-descend.**
Traditional approach: fly to B, hover, descend. Problem: during descent wind blows drone away from B. Glide approach eliminates this — drone is always moving toward B while descending.

**Body-frame conversion uses current yaw, not target yaw.**
In STABILIZE mode, RC pitch/roll commands move the drone relative to its **current** heading. The NED->body rotation must use the actual yaw from telemetry, not the target FLIGHT_YAW.

**PI-controllers for altitude and position.**
With SIM_WIND_SPD=3, a P-only controller has steady-state error. The integral term eliminates persistent wind-induced drift.

**Descent rate controller with feedforward.**
Without feedforward, the PI controller can't overcome hover throttle fast enough. FF term: `thr = HOVER - FF * |target_vz|` provides immediate throttle reduction proportional to desired descent rate.

**Wind-compensated aim point.**
During glide, the drone aims slightly upwind from B (15% of raw wind drift). This compensates for the residual drift that the PI controller can't fully eliminate.

**Constant Yaw = bearing A->B.**
Pre-computed bearing **A -> B ~ 228.5 deg** is used as `FLIGHT_YAW`. Pointing the nose toward B simplifies control: pitch drives forward progress, roll corrects lateral drift.

---

## Prerequisites

```
Python 3.8+
ArduCopter SITL (via Mission Planner or sim_vehicle.py)
```

```bash
pip install dronekit pymavlink
```

**Note on Python 3.10+:** dronekit may require patching `collections.MutableMapping` imports. The script includes an automatic compatibility patch.

---

## SITL Setup

### Option A — Mission Planner (Windows)

1. Open Mission Planner -> **Simulation** tab
2. Set **Home Location**: `50.450739, 30.461242`
3. Click **ArduCopter** -> wait for SITL to start
4. Mission Planner connects automatically on UDP 14550

### Option B — sim_vehicle.py (Linux/Mac)

```bash
cd ArduCopter
sim_vehicle.py \
  -v ArduCopter \
  --location=50.450739,30.461242,0,0 \
  --out=udp:127.0.0.1:14550 \
  --console --map
```

### Option C — Direct SITL binary (no MAVProxy)

```bash
arducopter --model + --speedup 1 \
  --defaults Tools/autotest/default_params/copter.parm \
  --home 50.450739,30.461242,0,0 -I0
# Connects on tcp:127.0.0.1:5760
```

---

## Running the script

```bash
# Default — connects to udp:127.0.0.1:14550
python flight.py

# Custom connection string
python flight.py --connect tcp:127.0.0.1:5760
```

---

## Tuning guide

| Parameter | Effect |
|-----------|--------|
| `HOVER_THROTTLE` | If drone climbs at 1500 -> lower; descends -> raise |
| `ALT_KP` / `ALT_KI` | Altitude response speed and steady-state accuracy |
| `NAV_KP` / `NAV_KI` | Position response speed and wind drift compensation |
| `YAW_KP` | Larger = faster yaw corrections |
| `GLIDE_ANGLE_DEG` | Steeper = shorter glide, faster flight; shallower = more gradual |
| `DESCENT_KP` / `DESCENT_KI` | Descent rate tracking accuracy |
| `DESCENT_FF` | Feedforward gain — critical for overcoming hover throttle |
| `WIND_SPD` / `WIND_DIR_DEG` | Must match SITL wind params for aim offset calculation |

In SITL the default hover throttle is usually **1500** (THR\_MID = 500 -> maps to RC 1500).
