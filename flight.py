#!/usr/bin/env python3
"""
╔══════════════════════════════════════════════════════════════════╗
║  ArduCopter STABILIZE Flight — RC Override Navigation           ║
║                                                                  ║
║  A (start) : 50.450739, 30.461242                               ║
║  B (target): 50.443326, 30.448078                               ║
║  Altitude  : 200 m                                               ║
║                                                                  ║
║  Phases:                                                         ║
║    1. Takeoff in STABILIZE → 200 m                              ║
║    2. Cruise A→B, constant YAW, RC override altitude hold       ║
║    3. Precision landing at B                                     ║
╚══════════════════════════════════════════════════════════════════╝
"""

import time
import math
import sys
import argparse
import collections
import collections.abc

# Patch for dronekit compatibility with Python 3.10+
for attr in ('MutableMapping', 'MutableSequence', 'Iterable', 'Mapping', 'Sequence'):
    if not hasattr(collections, attr):
        setattr(collections, attr, getattr(collections.abc, attr))

try:
    from dronekit import connect, VehicleMode
except ImportError:
    sys.exit("dronekit not found — run: pip install dronekit")

# ──────────────────────────────────────────────────────────────────
#  MISSION PARAMETERS
# ──────────────────────────────────────────────────────────────────
POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 200.0          # metres AGL

# RC neutral / limits
RC_MID  = 1500
RC_MIN  = 1100
RC_MAX  = 1900

# ── Altitude PI-controller ──────────────────────────────────────
HOVER_THROTTLE  = 1500      # throttle that ~= hover; tune if needed
ALT_KP          = 3.5       # (m error) → throttle delta
ALT_KI          = 0.15      # integral gain for steady-state altitude error
ALT_I_MAX       = 120.0     # anti-windup clamp for altitude integrator
ALT_MAX_DELTA   = 350       # clamp total throttle correction

# ── Position / navigation PI-controller ─────────────────────────
NAV_KP           = 3.0      # (m error) → RC delta
NAV_KI           = 0.15     # integral gain for wind-induced position drift
NAV_I_MAX        = 120.0    # anti-windup clamp for position integrator
NAV_MAX_CTRL     = 350      # max RC deviation for pitch/roll (needs to overcome wind)
SLOWDOWN_RADIUS  = 150.0    # m — start reducing speed here (larger = smoother approach)
ARRIVAL_RADIUS   = 15.0     # m — "at target B" (needs margin for wind)

# ── Yaw P-controller ───────────────────────────────────────────
YAW_KP           = 4.0      # (deg error) → CH4 delta (rate control, needs strong correction)
YAW_MAX_CTRL     = 150
YAW_DEADZONE     = 0.5      # degrees, tighter deadzone for precision

# ── Descent (wind compensation) ────────────────────────────────
WIND_SPD         = 3.0      # m/s — must match SIM_WIND_SPD
WIND_DIR         = 30.0     # deg — must match SIM_WIND_DIR (direction wind blows FROM)
FAST_DESCENT_THR = 1200     # aggressive throttle for fast descent (~8-10 m/s)
DESCENT_RATE_TARGET = -8.0  # m/s approx descent rate at FAST_DESCENT_THR
DESCENT_RATE_SLOW = -1.5    # m/s for final 15m
DESCENT_KP       = 5.0      # vz_error → throttle delta
DESCENT_KI       = 0.3      # integral for descent rate hold

# ── Loop / timeout ──────────────────────────────────────────────
LOOP_DT          = 0.08     # s  (~12.5 Hz control loop)
TIMEOUT_TAKEOFF  = 180      # s
TIMEOUT_CRUISE   = 900      # s
TIMEOUT_LAND     = 240      # s

# ── GPS sanity ──────────────────────────────────────────────────
HOME_MAX_DIST    = 50.0     # m — max acceptable distance from POINT_A at start


# ──────────────────────────────────────────────────────────────────
#  MATH HELPERS
# ──────────────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap180(deg):
    """Shortest signed angular difference → [-180, 180]."""
    return ((deg + 180.0) % 360.0) - 180.0

def bearing_deg(lat1, lon1, lat2, lon2):
    """True bearing from point-1 to point-2, 0..360° (0 = North)."""
    la1, la2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(la2)
    y = math.cos(la1)*math.sin(la2) - math.sin(la1)*math.cos(la2)*math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

def haversine_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres."""
    R = 6_371_000.0
    la1, la2 = math.radians(lat1), math.radians(lat2)
    dlat = la2 - la1
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(la1)*math.cos(la2)*math.sin(dlon/2)**2
    return R * 2.0 * math.asin(math.sqrt(max(0.0, a)))

def ned_offset_m(lat1, lon1, lat2, lon2):
    """(north, east) offset in metres from point-1 → point-2."""
    avg_lat = math.radians((lat1 + lat2) / 2.0)
    north = math.radians(lat2 - lat1) * 6_371_000.0
    east  = math.radians(lon2 - lon1) * 6_371_000.0 * math.cos(avg_lat)
    return north, east

def ned_to_body(north, east, yaw_deg):
    """
    Rotate NED offset to body frame using CURRENT drone yaw.
    Returns (forward, rightward) — positive fwd = ahead, positive rgt = right.
    """
    yaw = math.radians(yaw_deg)
    fwd = north * math.cos(yaw) + east * math.sin(yaw)
    rgt = -north * math.sin(yaw) + east * math.cos(yaw)
    return fwd, rgt


# ──────────────────────────────────────────────────────────────────
#  DRONEKIT / MAVLINK HELPERS
# ──────────────────────────────────────────────────────────────────

def set_mode(vehicle, mode_name, timeout=15):
    vehicle.mode = VehicleMode(mode_name)
    t0 = time.time()
    while vehicle.mode.name != mode_name:
        if time.time() - t0 > timeout:
            raise RuntimeError(f"Timed out waiting for mode {mode_name}")
        time.sleep(0.3)
    print(f"  Mode -> {mode_name}")

def arm_vehicle(vehicle, timeout=60):
    print("  Arming ...")
    vehicle.armed = True
    t0 = time.time()
    while not vehicle.armed:
        if time.time() - t0 > timeout:
            raise RuntimeError("Arm timeout")
        print("    ... waiting for arm", end="\r")
        time.sleep(1)
    print("  Armed!          ")

def disarm_vehicle(vehicle):
    vehicle.channels.overrides = {}
    time.sleep(0.3)
    vehicle.armed = False
    print("  Disarmed")

def rc_override(vehicle, roll=RC_MID, pitch=RC_MID, throttle=RC_MID, yaw=RC_MID):
    """Send RC channel overrides on CH1-4."""
    vehicle.channels.overrides = {
        '1': int(clamp(roll,     RC_MIN, RC_MAX)),  # CH1 Roll
        '2': int(clamp(pitch,    RC_MIN, RC_MAX)),  # CH2 Pitch
        '3': int(clamp(throttle, RC_MIN, RC_MAX)),  # CH3 Throttle
        '4': int(clamp(yaw,      RC_MIN, RC_MAX)),  # CH4 Yaw
    }

_last_valid_lat = 0.0
_last_valid_lon = 0.0
_last_valid_alt = 0.0

def get_state(vehicle):
    """Return (lat, lon, alt_rel, yaw_deg_0_360).
    Caches last valid GPS position to avoid jumping to (0,0) on GPS glitch."""
    global _last_valid_lat, _last_valid_lon, _last_valid_alt
    loc = vehicle.location.global_relative_frame
    lat = loc.lat
    lon = loc.lon
    alt = loc.alt

    # Guard against GPS returning None or (0,0) — use last valid position
    if lat is not None and lon is not None and abs(lat) > 1.0:
        _last_valid_lat = lat
        _last_valid_lon = lon
        _last_valid_alt = alt if alt is not None else _last_valid_alt
    else:
        lat = _last_valid_lat
        lon = _last_valid_lon
        alt = _last_valid_alt

    yaw_rad = vehicle.attitude.yaw     # -pi ... +pi
    yaw_d   = (math.degrees(yaw_rad) + 360.0) % 360.0
    return (lat, lon, alt, yaw_d)

def ch4_for_yaw(current_yaw, target_yaw):
    """CH4 value to rotate toward target_yaw (rate control in Stabilize)."""
    err = wrap180(target_yaw - current_yaw)
    if abs(err) < YAW_DEADZONE:
        return RC_MID
    return int(clamp(RC_MID + YAW_KP * err, RC_MIN, RC_MAX))


# ──────────────────────────────────────────────────────────────────
#  MAIN
# ──────────────────────────────────────────────────────────────────

def main(connection_string):
    # -- Pre-compute mission constants
    FLIGHT_YAW  = bearing_deg(*POINT_A, *POINT_B)
    DIST_TOTAL  = haversine_m(*POINT_A, *POINT_B)

    print("=" * 65)
    print("  DRONE STABILIZE FLIGHT  --  RC Override navigation")
    print(f"  A -> B bearing  : {FLIGHT_YAW:.1f} deg  (this is the CONSTANT YAW)")
    print(f"  Route distance : {DIST_TOTAL:.0f} m")
    print(f"  Target altitude: {TARGET_ALT:.0f} m")
    print("=" * 65)

    # -- Connect
    print(f"\nConnecting to {connection_string} ...")
    vehicle = connect(connection_string, wait_ready=True, timeout=60)
    print(f"Connected  |  FW: {vehicle.version}  |  Mode: {vehicle.mode.name}")

    # Set RC override timeout so overrides expire if script crashes
    try:
        vehicle.parameters['RC_OVERRIDE_TIME'] = 3.0
    except Exception:
        pass  # parameter may not exist on older firmware

    # -- Wait for GPS fix and EKF convergence
    print("\nWaiting for GPS fix and EKF convergence ...")
    t_wait = time.time()
    while time.time() - t_wait < 90:
        gps_fix = vehicle.gps_0.fix_type
        ekf = vehicle.ekf_ok
        sats = vehicle.gps_0.satellites_visible
        sys.stdout.write(
            f"\r  GPS fix={gps_fix}  sats={sats}  EKF={'OK' if ekf else 'converging...'}  "
            f"({int(time.time()-t_wait)}s)   "
        )
        sys.stdout.flush()
        if gps_fix >= 3 and ekf:
            print(f"\n  GPS and EKF ready ({int(time.time()-t_wait)}s)")
            break
        time.sleep(1)
    else:
        print("\n  WARNING: GPS/EKF timeout — attempting to continue anyway")

    lat0, lon0, alt0, _ = get_state(vehicle)
    home_dist = haversine_m(lat0, lon0, *POINT_A)
    print(f"  Current position: {lat0:.6f}, {lon0:.6f}")
    print(f"  Distance from Point A: {home_dist:.1f} m")
    if home_dist > HOME_MAX_DIST:
        print(f"  WARNING: Home is {home_dist:.0f}m from Point A (expected <{HOME_MAX_DIST}m).")
        print("  Make sure SITL home is set to Point A coordinates!")

    # -- Wind parameters (set one at a time with delays to avoid dronekit timeout)
    print("\nApplying SITL wind parameters ...")
    params = {
        'SIM_WIND_SPD': 3,
        'SIM_WIND_DIR': 30,
        'SIM_WIND_TURB': 2,
        'SIM_WIND_TURB_FREQ': 0.2,
        'ARMING_CHECK': 0,
    }
    for name, val in params.items():
        try:
            vehicle.parameters[name] = val
            print(f"  {name} = {val}")
            time.sleep(0.5)
        except Exception as e:
            print(f"  WARNING: Failed to set {name}: {e}")
    time.sleep(2)

    try:
        _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL)
    except KeyboardInterrupt:
        print("\n  INTERRUPTED by user")
    except Exception as e:
        print(f"\n  ERROR: {e}")
    finally:
        # Always clear RC overrides and disarm on exit
        print("\n  Cleaning up ...")
        try:
            vehicle.channels.overrides = {}
            time.sleep(0.3)
            if vehicle.armed:
                vehicle.armed = False
                time.sleep(2)
        except Exception:
            pass

        lat, lon, alt, _ = get_state(vehicle)
        final_dist = haversine_m(lat, lon, *POINT_B)

        print("\n" + "=" * 65)
        print("  FLIGHT COMPLETE")
        print(f"  Final position : {lat:.6f}, {lon:.6f}")
        print(f"  Final altitude : {alt:.2f} m")
        print(f"  Distance from B: {final_dist:.2f} m")
        print(f"  Constant yaw used: {FLIGHT_YAW:.1f} deg")
        print("=" * 65)

        vehicle.close()


def _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL):
    """Core flight logic — wrapped in try/finally by main()."""

    # ==============================================================
    # PHASE 1  --  TAKEOFF  (STABILIZE, RC override)
    # ==============================================================
    print(f"\n{'-'*65}")
    print("  PHASE 1 -- TAKEOFF in STABILIZE mode")
    print(f"{'-'*65}")

    set_mode(vehicle, "STABILIZE")
    arm_vehicle(vehicle)

    # Minimal throttle just after arm to prevent auto-disarm
    rc_override(vehicle, throttle=RC_MIN + 50)
    time.sleep(0.8)

    alt_integral = 0.0
    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        alt_err = TARGET_ALT - alt

        # Aggressive initial climb; PI control once airborne
        if alt < 3.0:
            thr = 1680
            alt_integral = 0.0
        else:
            alt_integral = clamp(alt_integral + alt_err * LOOP_DT,
                                 -ALT_I_MAX, ALT_I_MAX)
            delta = clamp(ALT_KP * alt_err + ALT_KI * alt_integral,
                          -ALT_MAX_DELTA, ALT_MAX_DELTA)
            thr = HOVER_THROTTLE + delta

        # Rotate toward FLIGHT_YAW during climb
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        rc_override(vehicle, roll=RC_MID, pitch=RC_MID, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [TKOFF]  Alt {alt:6.1f}/{TARGET_ALT:.0f} m  "
            f"Thr {int(thr)}  Yaw {yaw:6.1f} -> {FLIGHT_YAW:.1f} deg   "
        )
        sys.stdout.flush()

        if alt >= TARGET_ALT - 1.0:
            print(f"\n  Target altitude reached -- {alt:.1f} m")
            break

        if time.time() - t0 > TIMEOUT_TAKEOFF:
            print("\n  Takeoff timeout -- aborting")
            disarm_vehicle(vehicle)
            vehicle.close()
            sys.exit(1)

        time.sleep(LOOP_DT)

    # ==============================================================
    # PHASE 2  --  CRUISE  A -> B
    # ==============================================================
    print(f"\n{'-'*65}")
    print(f"  PHASE 2 -- CRUISE  ->  Point B  (~{DIST_TOTAL:.0f} m)")
    print(f"  Constant YAW = {FLIGHT_YAW:.1f} deg  throughout entire flight")
    print(f"{'-'*65}")

    # Reset integrators for cruise phase
    alt_integral = 0.0
    nav_fwd_integral = 0.0
    nav_rgt_integral = 0.0

    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        dist = haversine_m(lat, lon, *POINT_B)

        # -- Altitude hold (PI-controller)
        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT,
                             -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(
            HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
            RC_MIN, RC_MAX))

        # -- Position -> body-frame error (using CURRENT yaw, not target)
        n_err, e_err = ned_offset_m(lat, lon, *POINT_B)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)

        # Speed scaling: gentle near target, but keep full authority if overshooting
        # Detect overshoot: if bearing to B differs from FLIGHT_YAW by >90 deg, we passed it
        bearing_to_b = bearing_deg(lat, lon, *POINT_B)
        angle_diff = abs(wrap180(bearing_to_b - FLIGHT_YAW))
        is_overshoot = angle_diff > 90

        if is_overshoot or dist > SLOWDOWN_RADIUS:
            max_ctrl = NAV_MAX_CTRL  # full authority
        else:
            spd_scale = dist / SLOWDOWN_RADIUS
            max_ctrl = max(NAV_MAX_CTRL * spd_scale, 80)  # min 80 for wind compensation

        # PI control for position
        nav_fwd_integral = clamp(nav_fwd_integral + fwd * LOOP_DT,
                                 -NAV_I_MAX, NAV_I_MAX)
        nav_rgt_integral = clamp(nav_rgt_integral + rgt * LOOP_DT,
                                 -NAV_I_MAX, NAV_I_MAX)

        pitch_delta = clamp(NAV_KP * fwd + NAV_KI * nav_fwd_integral,
                            -max_ctrl, max_ctrl)
        roll_delta  = clamp(NAV_KP * rgt + NAV_KI * nav_rgt_integral,
                            -max_ctrl, max_ctrl)

        # Reset integrators when close to avoid overshoot
        if dist < ARRIVAL_RADIUS * 3:
            nav_fwd_integral *= 0.9
            nav_rgt_integral *= 0.9

        # ArduCopter convention:
        #   CH2 < 1500 -> nose down  -> fly forward
        #   CH1 > 1500 -> roll right -> move right
        ch2 = RC_MID - int(pitch_delta)
        ch1 = RC_MID + int(roll_delta)

        # -- Constant yaw maintenance
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [CRUISE]  Dist {dist:7.1f} m  Alt {alt:6.1f} m  "
            f"Yaw {yaw:6.1f} deg  fwd {fwd:+6.1f}  rgt {rgt:+6.1f}   "
        )
        sys.stdout.flush()

        if dist <= ARRIVAL_RADIUS:
            print(f"\n  Arrived at Point B  (dist = {dist:.2f} m)")
            break

        if time.time() - t0 > TIMEOUT_CRUISE:
            print(f"\n  Cruise timeout -- proceeding to land  (dist = {dist:.1f} m)")
            break

        time.sleep(LOOP_DT)

    # ==============================================================
    # PHASE 3  --  WIND COMPENSATION LANDING at B
    # ==============================================================
    print(f"\n{'-'*65}")
    print("  PHASE 3 -- WIND COMPENSATION LANDING at Point B")
    print(f"{'-'*65}")

    # STEP 1: Calculate descent start point (upwind from B)
    descent_time = TARGET_ALT / abs(DESCENT_RATE_TARGET)
    wind_offset = WIND_SPD * descent_time  # how far wind will push during descent

    # Wind blows FROM WIND_DIR, so drift is in direction WIND_DIR + 180
    # We need to start UPWIND = opposite of drift = toward WIND_DIR
    wind_to_rad = math.radians(WIND_DIR)  # direction wind comes FROM = where we go
    offset_north = wind_offset * math.cos(wind_to_rad)
    offset_east = wind_offset * math.sin(wind_to_rad)

    descent_lat = POINT_B[0] + offset_north / 111320.0
    descent_lon = POINT_B[1] + offset_east / (111320.0 * math.cos(math.radians(POINT_B[0])))
    DESCENT_POINT = (descent_lat, descent_lon)

    descent_dist = haversine_m(*DESCENT_POINT, *POINT_B)
    print(f"  Wind: {WIND_SPD} m/s from {WIND_DIR} deg")
    print(f"  Estimated descent time: {descent_time:.0f}s, wind offset: {wind_offset:.0f}m")
    print(f"  Descent start point: {descent_lat:.6f}, {descent_lon:.6f}")
    print(f"  Distance from B to descent point: {descent_dist:.0f}m")

    # STEP 2: Fly to descent point (horizontal, maintaining altitude)
    print("  Flying to descent start point ...")
    nav_fwd_integral = 0.0
    nav_rgt_integral = 0.0
    alt_integral = 0.0

    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        dist_to_dp = haversine_m(lat, lon, *DESCENT_POINT)

        # Altitude hold
        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                        RC_MIN, RC_MAX))

        # Navigate to descent point
        n_err, e_err = ned_offset_m(lat, lon, *DESCENT_POINT)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)

        nav_fwd_integral = clamp(nav_fwd_integral + fwd * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)
        nav_rgt_integral = clamp(nav_rgt_integral + rgt * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)

        spd_scale = min(1.0, dist_to_dp / 40.0)
        max_c = max(NAV_MAX_CTRL * spd_scale, 80)
        ch2 = RC_MID - int(clamp(NAV_KP * fwd + NAV_KI * nav_fwd_integral, -max_c, max_c))
        ch1 = RC_MID + int(clamp(NAV_KP * rgt + NAV_KI * nav_rgt_integral, -max_c, max_c))
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)

        dist_b = haversine_m(lat, lon, *POINT_B)
        sys.stdout.write(
            f"\r  [->DP]   Dist-DP {dist_to_dp:5.1f}m  Dist-B {dist_b:5.1f}m  "
            f"Alt {alt:.1f}m  Yaw {yaw:.1f} deg   "
        )
        sys.stdout.flush()

        if dist_to_dp <= 15.0:
            print(f"\n  Reached descent point (dist = {dist_to_dp:.1f}m)")
            break

        if time.time() - t0 > 180:
            print(f"\n  Descent point timeout (dist = {dist_to_dp:.1f}m) -- descending anyway")
            break

        time.sleep(LOOP_DT)

    # STEP 2.5: Stabilize at descent point for 3 seconds
    print("  Stabilising at descent point ...")
    t_hold = time.time()
    while time.time() - t_hold < 3.0:
        lat, lon, alt, yaw = get_state(vehicle)
        n_err, e_err = ned_offset_m(lat, lon, *DESCENT_POINT)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)
        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                        RC_MIN, RC_MAX))
        ch2 = RC_MID - int(clamp(NAV_KP * fwd, -80, 80))
        ch1 = RC_MID + int(clamp(NAV_KP * rgt, -80, 80))
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)
        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)
        time.sleep(LOOP_DT)

    # STEP 3: Fast descent — let wind carry drone toward B
    print("  DESCENDING (wind compensation) ...")
    prev_alt = alt
    vz_filtered = 0.0
    t_prev = time.time()
    t0 = time.time()

    while True:
        t_now = time.time()
        dt_actual = max(t_now - t_prev, 0.01)
        t_prev = t_now

        lat, lon, alt, yaw = get_state(vehicle)
        dist_b = haversine_m(lat, lon, *POINT_B)

        # Minimal horizontal correction toward B during descent
        n_err, e_err = ned_offset_m(lat, lon, *POINT_B)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)
        land_nav_max = 50  # light touch
        ch2 = RC_MID - int(clamp(NAV_KP * fwd * 0.3, -land_nav_max, land_nav_max))
        ch1 = RC_MID + int(clamp(NAV_KP * rgt * 0.3, -land_nav_max, land_nav_max))

        # Fast descent throttle with gentle slowdown near ground
        vz_raw = (alt - prev_alt) / dt_actual
        prev_alt = alt
        vz_filtered = 0.7 * vz_filtered + 0.3 * vz_raw

        if alt > 15:
            land_thr = FAST_DESCENT_THR  # fast drop
        else:
            # Slow down for final approach
            target_vz = DESCENT_RATE_SLOW
            vz_err = target_vz - vz_filtered
            land_thr = int(clamp(HOVER_THROTTLE + DESCENT_KP * vz_err,
                                 RC_MIN, RC_MAX))

        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)
        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=land_thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [LAND]   Alt {alt:6.2f}m  Dist-B {dist_b:5.2f}m  "
            f"Thr {land_thr}  Vz {vz_filtered:+.1f} m/s  Yaw {yaw:.1f} deg   "
        )
        sys.stdout.flush()

        if alt < 0.25 or not vehicle.armed:
            print(f"\n  Touchdown!  final dist from B = {dist_b:.2f} m")
            break

        if time.time() - t0 > TIMEOUT_LAND:
            print(f"\n  Landing timeout  (alt = {alt:.1f} m)")
            break

        time.sleep(LOOP_DT)

    # Flight completed normally — cleanup handled by finally in main()


# ──────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ArduCopter STABILIZE flight A->B via RC Override"
    )
    parser.add_argument(
        "--connect",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string (default: udp:127.0.0.1:14550)"
    )
    args = parser.parse_args()
    main(args.connect)

