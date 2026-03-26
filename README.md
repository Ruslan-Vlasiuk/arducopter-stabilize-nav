# ArduCopter STABILIZE Flight — RC Override Navigation

Autonomous A->B flight in **STABILIZE mode** using **RC Override**.
No assisted flight modes — altitude, heading and position are controlled entirely through raw RC channel overrides.

| Parameter | Value |
|-----------|-------|
| Point A (start) | `50.450739, 30.461242` |
| Point B (target) | `50.443326, 30.448078` |
| Cruise altitude | **200 m** |
| Flight mode | **STABILIZE** throughout |
| Constant Yaw | **~228.6 deg** (bearing A->B, fixed for the entire flight) |
| Wind (SITL) | SPD=3, DIR=30 deg, TURB=2, TURB\_FREQ=0.2 |

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
|  +----------------+  +--------------------+    |
|  |  Altitude PI   |  |  Position PI       |    |
|  |  error -> CH3  |  |  NED -> body frame |    |
|  |  (throttle)    |  |  fwd/rgt -> CH2/CH1|    |
|  +----------------+  +--------------------+    |
|                                                |
|  +----------------------------------------+    |
|  |  Yaw P  (rate control in STABILIZE)    |    |
|  |  error -> CH4  ->  holds FLIGHT_YAW   |    |
|  +----------------------------------------+    |
+------------------------------------------------+
        |
        v
  RC Override CH1-4  ->  ArduCopter STABILIZE
```

### Flight phases

| Phase | Description |
|-------|-------------|
| **1. Takeoff** | STABILIZE mode, CH3 drives aggressive climb to 200 m with PI altitude hold. Yaw rotates to bearing A->B. |
| **2. Cruise** | PI-controller on altitude (CH3), NED position error rotated to body frame using **current yaw** -> CH1/CH2 with PI control. Yaw locked to **228.6 deg** via CH4. Speed scales down inside 60 m radius. |
| **3. Precision landing** | 4 s momentum-zeroing hover with PI position hold, then descent rate PI-controller (faster high up, gentle last 10 m), with continuous horizontal PI correction over B. |

### Key design decisions

**Body-frame conversion uses current yaw, not target yaw.**
In STABILIZE mode, RC pitch/roll commands move the drone relative to its **current** heading. The NED->body rotation must use the actual yaw from telemetry, not the target FLIGHT_YAW.

**PI-controllers (not just P) for altitude and position.**
With SIM_WIND_SPD=3 and SIM_WIND_DIR=30, a P-only controller has steady-state error. The integral term eliminates persistent wind-induced drift in both altitude and horizontal position.

**Descent rate controller instead of hardcoded throttle.**
Landing uses a PI controller targeting a specific descent rate (-2.5 m/s, slowing to -0.8 m/s below 10m) rather than hardcoded throttle values, making it robust to different vehicle configurations.

**Constant Yaw = bearing A->B.**
Pre-computed bearing **A -> B ~ 228.6 deg** is used as `FLIGHT_YAW`. Pointing the nose toward B simplifies control: pitch drives forward progress, roll corrects lateral drift.

**GPS and home position verification.**
Before flight, the script checks GPS fix quality and verifies the drone's starting position matches Point A.

---

## Prerequisites

```
Python 3.8+
ArduCopter SITL (via Mission Planner or sim_vehicle.py)
```

```bash
pip install dronekit pymavlink
```

**Note on Python 3.10+:** dronekit may require patching `collections.MutableMapping` imports. If you get an import error, edit the dronekit source to use `collections.abc.MutableMapping`.

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

The `--location` flag sets SITL home = **Point A**.

---

## Running the script

```bash
# Default — connects to udp:127.0.0.1:14550
python flight.py

# Custom connection string
python flight.py --connect tcp:127.0.0.1:5762

# If Mission Planner is already open and SITL is running,
# you may need to use port 14551 (MP occupies 14550):
python flight.py --connect udp:127.0.0.1:14551
```

---

## Tuning guide

| Parameter | Effect |
|-----------|--------|
| `HOVER_THROTTLE` | If drone climbs at 1500 -> lower; descends -> raise |
| `ALT_KP` / `ALT_KI` | Altitude response speed and steady-state accuracy |
| `NAV_KP` / `NAV_KI` | Position response speed and wind drift compensation |
| `SLOWDOWN_RADIUS` | Larger = starts slowing down earlier |
| `YAW_KP` | Larger = faster yaw corrections |
| `DESCENT_RATE` | Target descent rate in m/s (negative = down) |
| `DESCENT_KP` / `DESCENT_KI` | Descent rate tracking accuracy |

In SITL the default hover throttle is usually **1500** (THR\_MID = 500 -> maps to RC 1500).
If your SITL vehicle doesn't hover at 1500, adjust `HOVER_THROTTLE` first.
