# ArduCopter STABILIZE Flight — RC Override Navigation (Cascaded Velocity Control)

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
| 1 | 0.05 m | 182 s |
| 2 | 0.12 m | 184 s |
| 3 | 0.11 m | 184 s |
| 4 | 0.01 m | 182 s |
| 5 | 0.04 m | 182 s |
| **Average** | **0.07 m** | **182.8 s** |

---

## How it works

### Architecture

```
GPS telemetry + EKF velocity (dronekit)
        |
        v
+--------------------------------------------------+
|  Control loop  (~12.5 Hz)                         |
|                                                   |
|  Phase 1: TAKEOFF                                 |
|    Altitude PI -> CH3 (throttle)                  |
|    Yaw P -> CH4 (rotate to bearing A->B)          |
|                                                   |
|  Phase 2: CRUISE + GLIDE + FLARE                  |
|    Outer loop: position P -> desired velocity     |
|    Inner loop: velocity PI -> CH1/CH2             |
|    EMA low-pass filter on body-frame velocities   |
|    Altitude: PI (cruise) / PI+FF descent (glide)  |
|    Yaw P -> CH4                                   |
+--------------------------------------------------+
        |
        v
  RC Override CH1-4  ->  ArduCopter STABILIZE
```

### Cascaded velocity controller (key innovation)

Traditional approach uses **position PI** directly -> RC commands. This reacts slowly to wind gusts (must wait for position change).

Our **cascaded controller** uses two loops:
1. **Outer loop (position P):** position error -> desired velocity (`POS_KP * error`)
2. **Inner loop (velocity PI):** velocity error -> RC pitch/roll (`VEL_KP * vel_err + VEL_KI * integral`)

The inner velocity loop reacts **immediately** to wind gusts (velocity changes before position does), providing much tighter position hold.

### Glide approach + Flare

Instead of flying to B at altitude and then descending (which causes wind drift),
the drone follows an **18-degree glide slope** with a **35-degree steep final slope**:

```
200m ___________
               \  glide_start = 615m from B
                \
                 \  18 deg slope
                  \
                   \_ 35 deg steep (last 20m)
                     |
                     | FLARE (alt < 2m)
                     v B (ground)
```

- Glide: continuous approach to B while descending (no hover phase)
- Steep slope at dist < 20m: preserves altitude for position correction
- Flare at alt < 2m: velocity controller holds position over B during final descent
- Wind compensation: upwind aim offset (~15% of estimated drift for dist > 30m, fixed 4.5m for 5-30m)

### Flight phases

| Phase | Description |
|-------|-------------|
| **1. Takeoff** | STABILIZE mode, aggressive climb to 200m with PI altitude hold. Yaw rotates to bearing A->B. |
| **2. Cruise** | PI altitude hold at 200m, cascaded velocity nav (position P -> velocity PI) toward B. |
| **3. Glide** | When dist <= 615m, target altitude follows glide slope. Descent rate PI + feedforward. Steep 35-deg slope in final 20m. |
| **4. Flare** | When alt < 2m and dist < 10m: slow descent (Vz=-0.8), velocity controller holds position over B for precision touchdown. |

### Key design decisions

**Cascaded velocity control instead of direct position PI.**
Position PI reacts to position changes (slow). Velocity PI reacts to velocity changes (fast). Wind gust changes velocity before position — cascaded control catches it 5-10x faster.

**Glide approach instead of hover-then-descend.**
Traditional: fly to B, hover, descend. Problem: during descent wind blows drone away from B. Glide approach eliminates this — drone is always moving toward B while descending.

**Flare phase at 2m altitude.**
At very low altitude, the steep slope transitions to a slow vertical descent while the velocity controller maintains position directly over B. This gives maximum precision at touchdown.

**Body-frame conversion uses current yaw, not target yaw.**
In STABILIZE mode, RC pitch/roll commands move the drone relative to its **current** heading. The NED->body rotation must use the actual yaw from telemetry, not the target FLIGHT_YAW.

**EMA low-pass filter on velocities.**
Raw EKF velocity has noise. The EMA filter (alpha=0.25) smooths body-frame velocities for stable control without significant lag.

**Wind-compensated aim point.**
During glide, the drone aims upwind from B (15% of estimated wind drift at dist > 30m, fixed 4.5m offset at 5-30m). This pre-compensates for residual drift the PI controller can't fully eliminate.

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
| `POS_KP` | Outer loop: m/s of desired velocity per m of position error |
| `VEL_KP` / `VEL_KI` | Inner loop: RC delta per m/s of velocity error |
| `MAX_VEL_*` | Speed limits for cruise/glide/low altitude |
| `YAW_KP` | Larger = faster yaw corrections |
| `GLIDE_ANGLE_DEG` | Steeper = shorter glide, faster flight; shallower = more gradual |
| `DESCENT_KP` / `DESCENT_KI` | Descent rate tracking accuracy |
| `DESCENT_FF` | Feedforward gain — critical for overcoming hover throttle |
| `WIND_SPD` / `WIND_DIR_DEG` | Must match SITL wind params for aim offset calculation |
| `VEL_EMA_ALPHA` | Velocity filter: lower = smoother, higher = more responsive |

In SITL the default hover throttle is usually **1500** (THR\_MID = 500 -> maps to RC 1500).
