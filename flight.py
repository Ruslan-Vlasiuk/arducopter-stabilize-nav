#!/usr/bin/env python3
"""
ArduCopter STABILIZE Flight — RC Override Navigation

A (start) : 50.450739, 30.461242
B (target): 50.443326, 30.448078
Altitude  : 200 m

Phases:
  1. Takeoff in STABILIZE -> 200 m
  2. Cruise A->B (CRUISE + APPROACH_FINE subphases)
  3. Wind compensation landing at B
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
    sys.exit("dronekit not found -- run: pip install dronekit")

# ==================================================================
#  MISSION PARAMETERS
# ==================================================================
POINT_A = (50.450739, 30.461242)
POINT_B = (50.443326, 30.448078)
TARGET_ALT = 200.0

RC_MID = 1500
RC_MIN = 1100
RC_MAX = 1900

# -- Altitude PI
HOVER_THROTTLE = 1500
ALT_KP = 5.0
ALT_KI = 0.05
ALT_I_MAX = 200.0
ALT_MAX_DELTA = 380

# -- Navigation PI (CRUISE phase: far from B)
CRUISE_NAV_KP = 3.0
CRUISE_NAV_MAX = 300   # full speed pitch delta

# -- Navigation PI (APPROACH_FINE phase: near B)
FINE_NAV_KP = 2.0
FINE_NAV_MAX = 100     # slow approach pitch delta
FINE_RADIUS = 80.0     # switch to APPROACH_FINE below this distance

NAV_KI = 0.15
NAV_I_MAX = 120.0
ARRIVAL_RADIUS = 15.0

# -- Yaw P
YAW_KP = 4.0
YAW_DEADZONE = 0.5

# -- Wind compensation descent
WIND_SPD = 3.0
WIND_DIR = 30.0
FAST_DESCENT_THR = 1300   # ~8 m/s descent
DESCENT_RATE_EST = 8.0    # estimated m/s at FAST_DESCENT_THR
FLARE_ALT = 3.0           # start flare at this altitude
FLARE_THR = 1420           # soft landing throttle

# -- Timeouts
LOOP_DT = 0.08
TIMEOUT_TAKEOFF = 180
TIMEOUT_CRUISE = 900
TIMEOUT_LAND = 60
HOME_MAX_DIST = 50.0


# ==================================================================
#  MATH HELPERS
# ==================================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def wrap180(deg):
    return ((deg + 180.0) % 360.0) - 180.0

def bearing_deg(lat1, lon1, lat2, lon2):
    la1, la2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    x = math.sin(dl) * math.cos(la2)
    y = math.cos(la1)*math.sin(la2) - math.sin(la1)*math.cos(la2)*math.cos(dl)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    la1, la2 = math.radians(lat1), math.radians(lat2)
    dlat = la2 - la1
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(la1)*math.cos(la2)*math.sin(dlon/2)**2
    return R * 2.0 * math.asin(math.sqrt(max(0.0, a)))

def ned_offset_m(lat1, lon1, lat2, lon2):
    avg_lat = math.radians((lat1 + lat2) / 2.0)
    north = math.radians(lat2 - lat1) * 6_371_000.0
    east = math.radians(lon2 - lon1) * 6_371_000.0 * math.cos(avg_lat)
    return north, east

def ned_to_body(north, east, yaw_deg):
    yaw = math.radians(yaw_deg)
    fwd = north * math.cos(yaw) + east * math.sin(yaw)
    rgt = -north * math.sin(yaw) + east * math.cos(yaw)
    return fwd, rgt


# ==================================================================
#  VEHICLE HELPERS
# ==================================================================

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
        sys.stdout.write("    ... waiting for arm\r")
        sys.stdout.flush()
        time.sleep(1)
    print("  Armed!          ")

def rc_override(vehicle, roll=RC_MID, pitch=RC_MID, throttle=RC_MID, yaw=RC_MID):
    vehicle.channels.overrides = {
        '1': int(clamp(roll, RC_MIN, RC_MAX)),
        '2': int(clamp(pitch, RC_MIN, RC_MAX)),
        '3': int(clamp(throttle, RC_MIN, RC_MAX)),
        '4': int(clamp(yaw, RC_MIN, RC_MAX)),
    }

_last_valid_lat = 0.0
_last_valid_lon = 0.0
_last_valid_alt = 0.0

def get_state(vehicle):
    global _last_valid_lat, _last_valid_lon, _last_valid_alt
    loc = vehicle.location.global_relative_frame
    lat, lon, alt = loc.lat, loc.lon, loc.alt
    if lat is not None and lon is not None and abs(lat) > 1.0:
        _last_valid_lat = lat
        _last_valid_lon = lon
        _last_valid_alt = alt if alt is not None else _last_valid_alt
    else:
        lat, lon, alt = _last_valid_lat, _last_valid_lon, _last_valid_alt
    yaw_d = (math.degrees(vehicle.attitude.yaw) + 360.0) % 360.0
    return (lat, lon, alt, yaw_d)

def get_h_speed(vehicle):
    """Horizontal speed from vehicle.velocity [vx, vy, vz] in m/s."""
    v = vehicle.velocity
    if v and len(v) >= 2 and v[0] is not None and v[1] is not None:
        return math.sqrt(v[0]**2 + v[1]**2)
    return 0.0

def ch4_for_yaw(current_yaw, target_yaw):
    err = wrap180(target_yaw - current_yaw)
    if abs(err) < YAW_DEADZONE:
        return RC_MID
    return int(clamp(RC_MID + YAW_KP * err, RC_MIN, RC_MAX))


# ==================================================================
#  MAIN
# ==================================================================

def main(connection_string):
    FLIGHT_YAW = bearing_deg(*POINT_A, *POINT_B)
    DIST_TOTAL = haversine_m(*POINT_A, *POINT_B)
    flight_start = time.time()

    print("=" * 65)
    print("  DRONE STABILIZE FLIGHT -- RC Override navigation")
    print(f"  A -> B bearing  : {FLIGHT_YAW:.1f} deg (CONSTANT YAW)")
    print(f"  Route distance : {DIST_TOTAL:.0f} m")
    print(f"  Target altitude: {TARGET_ALT:.0f} m")
    print("=" * 65)

    print(f"\nConnecting to {connection_string} ...")
    vehicle = connect(connection_string, wait_ready=True, timeout=60)
    print(f"Connected | FW: {vehicle.version} | Mode: {vehicle.mode.name}")

    try:
        vehicle.parameters['RC_OVERRIDE_TIME'] = 3.0
    except Exception:
        pass

    # -- Wait for GPS/EKF
    print("\nWaiting for GPS fix and EKF convergence ...")
    t_wait = time.time()
    while time.time() - t_wait < 90:
        gps_fix = vehicle.gps_0.fix_type
        ekf = vehicle.ekf_ok
        sats = vehicle.gps_0.satellites_visible
        sys.stdout.write(
            f"\r  GPS fix={gps_fix} sats={sats} EKF={'OK' if ekf else 'converging...'} "
            f"({int(time.time()-t_wait)}s)   "
        )
        sys.stdout.flush()
        if gps_fix >= 3 and ekf:
            print(f"\n  GPS and EKF ready ({int(time.time()-t_wait)}s)")
            break
        time.sleep(1)
    else:
        print("\n  WARNING: GPS/EKF timeout")

    lat0, lon0, _, _ = get_state(vehicle)
    home_dist = haversine_m(lat0, lon0, *POINT_A)
    print(f"  Position: {lat0:.6f}, {lon0:.6f} ({home_dist:.1f}m from A)")

    # -- Wind parameters
    print("\nApplying SITL wind parameters ...")
    for name, val in {'SIM_WIND_SPD': 3, 'SIM_WIND_DIR': 30,
                      'SIM_WIND_TURB': 2, 'SIM_WIND_TURB_FREQ': 0.2,
                      'ARMING_CHECK': 0}.items():
        try:
            vehicle.parameters[name] = val
            print(f"  {name} = {val}")
            time.sleep(0.5)
        except Exception as e:
            print(f"  WARNING: {name}: {e}")
    time.sleep(2)

    try:
        _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL, flight_start)
    except KeyboardInterrupt:
        print("\n  INTERRUPTED by user")
    except Exception as e:
        print(f"\n  ERROR: {e}")
        import traceback; traceback.print_exc()
    finally:
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
        elapsed = time.time() - flight_start

        print("\n" + "=" * 65)
        print("  RESULT")
        print(f"  Final position : {lat:.6f}, {lon:.6f}")
        print(f"  Final altitude : {alt:.2f} m")
        print(f"  Distance from B: {final_dist:.2f} m")
        print(f"  Constant Yaw   : {FLIGHT_YAW:.1f} deg")
        print(f"  Flight time    : {elapsed:.0f} s")
        print("=" * 65)

        vehicle.close()


def _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL, flight_start):

    # ==============================================================
    # PHASE 1 -- TAKEOFF
    # ==============================================================
    print(f"\n{'-'*65}")
    print("  PHASE 1 -- TAKEOFF in STABILIZE mode")
    print(f"{'-'*65}")

    set_mode(vehicle, "STABILIZE")
    arm_vehicle(vehicle)

    rc_override(vehicle, throttle=RC_MIN + 50)
    time.sleep(0.8)

    alt_integral = 0.0
    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        alt_err = TARGET_ALT - alt

        if alt < 3.0:
            thr = 1680
            alt_integral = 0.0
        else:
            alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
            delta = clamp(ALT_KP * alt_err + ALT_KI * alt_integral, -ALT_MAX_DELTA, ALT_MAX_DELTA)
            thr = HOVER_THROTTLE + delta

        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)
        rc_override(vehicle, roll=RC_MID, pitch=RC_MID, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [TKOFF] Alt:{alt:6.1f}/{TARGET_ALT:.0f}m "
            f"Yaw:{yaw:5.1f}->{FLIGHT_YAW:.1f} Thr:{int(thr)}   "
        )
        sys.stdout.flush()

        if alt >= TARGET_ALT - 1.0:
            print(f"\n  Target altitude reached -- {alt:.1f} m")
            break
        if time.time() - t0 > TIMEOUT_TAKEOFF:
            raise RuntimeError("Takeoff timeout")
        time.sleep(LOOP_DT)

    # ==============================================================
    # PHASE 2 -- CRUISE + APPROACH_FINE
    # ==============================================================
    print(f"\n{'-'*65}")
    print(f"  PHASE 2 -- CRUISE -> Point B (~{DIST_TOTAL:.0f} m)")
    print(f"  Constant YAW = {FLIGHT_YAW:.1f} deg")
    print(f"{'-'*65}")

    alt_integral = 0.0
    nav_fwd_integral = 0.0
    nav_rgt_integral = 0.0

    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        dist = haversine_m(lat, lon, *POINT_B)
        h_speed = get_h_speed(vehicle)

        # Altitude PI
        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                        RC_MIN, RC_MAX))

        # Position -> body frame
        n_err, e_err = ned_offset_m(lat, lon, *POINT_B)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)

        # Select subphase
        if dist > FINE_RADIUS:
            phase_name = "CRUISE"
            nav_kp = CRUISE_NAV_KP
            max_ctrl = CRUISE_NAV_MAX
        else:
            phase_name = "FINE"
            nav_kp = FINE_NAV_KP
            max_ctrl = FINE_NAV_MAX

        # Overshoot detection: full authority if we passed B
        bearing_to_b = bearing_deg(lat, lon, *POINT_B)
        if abs(wrap180(bearing_to_b - FLIGHT_YAW)) > 90:
            max_ctrl = CRUISE_NAV_MAX  # full power to return

        # PI position control
        nav_fwd_integral = clamp(nav_fwd_integral + fwd * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)
        nav_rgt_integral = clamp(nav_rgt_integral + rgt * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)

        pitch_delta = clamp(nav_kp * fwd + NAV_KI * nav_fwd_integral, -max_ctrl, max_ctrl)
        roll_delta = clamp(nav_kp * rgt + NAV_KI * nav_rgt_integral, -max_ctrl, max_ctrl)

        if dist < ARRIVAL_RADIUS * 3:
            nav_fwd_integral *= 0.9
            nav_rgt_integral *= 0.9

        ch2 = RC_MID - int(pitch_delta)
        ch1 = RC_MID + int(roll_delta)
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [{phase_name:6s}] Dist:{dist:7.1f}m Alt:{alt:6.1f}m "
            f"Yaw:{yaw:5.1f}/{FLIGHT_YAW:.1f} hSpd:{h_speed:4.1f}m/s Thr:{thr}   "
        )
        sys.stdout.flush()

        if dist <= ARRIVAL_RADIUS:
            print(f"\n  Arrived near B (dist={dist:.1f}m, hSpd={h_speed:.1f}m/s)")
            # Wait for h_speed to drop
            if h_speed < 3.0:
                print(f"  Speed OK ({h_speed:.1f} m/s) -- proceeding to land")
                break
        if time.time() - t0 > TIMEOUT_CRUISE:
            print(f"\n  Cruise timeout (dist={dist:.1f}m)")
            break
        time.sleep(LOOP_DT)

    # ==============================================================
    # PHASE 3 -- WIND COMPENSATION LANDING
    # ==============================================================
    print(f"\n{'-'*65}")
    print("  PHASE 3 -- WIND COMPENSATION LANDING")
    print(f"{'-'*65}")

    # STEP 1: Calculate descent start point
    descent_time = TARGET_ALT / DESCENT_RATE_EST
    wind_offset = WIND_SPD * descent_time

    # Start UPWIND from B (wind blows FROM WIND_DIR, drift is opposite)
    wind_to_rad = math.radians(WIND_DIR)
    offset_north = wind_offset * math.cos(wind_to_rad)
    offset_east = wind_offset * math.sin(wind_to_rad)

    descent_lat = POINT_B[0] + offset_north / 111320.0
    descent_lon = POINT_B[1] + offset_east / (111320.0 * math.cos(math.radians(POINT_B[0])))
    DESCENT_POINT = (descent_lat, descent_lon)
    dp_dist = haversine_m(*DESCENT_POINT, *POINT_B)

    print(f"  DESCENT_POINT: {descent_lat:.6f}, {descent_lon:.6f}")
    print(f"  Wind offset: {offset_north:.1f}m N, {offset_east:.1f}m E")
    print(f"  Expected descent time: {descent_time:.1f} s")
    print(f"  Distance DP -> B: {dp_dist:.0f}m")

    # STEP 2: Fly to descent point
    print("  Flying to descent point ...")
    nav_fwd_integral = 0.0
    nav_rgt_integral = 0.0

    t0 = time.time()
    while True:
        lat, lon, alt, yaw = get_state(vehicle)
        dist_dp = haversine_m(lat, lon, *DESCENT_POINT)
        dist_b = haversine_m(lat, lon, *POINT_B)

        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                        RC_MIN, RC_MAX))

        n_err, e_err = ned_offset_m(lat, lon, *DESCENT_POINT)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)

        nav_fwd_integral = clamp(nav_fwd_integral + fwd * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)
        nav_rgt_integral = clamp(nav_rgt_integral + rgt * LOOP_DT, -NAV_I_MAX, NAV_I_MAX)

        spd = min(1.0, dist_dp / 40.0)
        mc = max(CRUISE_NAV_MAX * spd, 80)
        ch2 = RC_MID - int(clamp(CRUISE_NAV_KP * fwd + NAV_KI * nav_fwd_integral, -mc, mc))
        ch1 = RC_MID + int(clamp(CRUISE_NAV_KP * rgt + NAV_KI * nav_rgt_integral, -mc, mc))
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [->DP] Dist-DP:{dist_dp:5.1f}m Dist-B:{dist_b:5.1f}m "
            f"Alt:{alt:.1f}m Yaw:{yaw:.1f}   "
        )
        sys.stdout.flush()

        if dist_dp <= 15.0:
            print(f"\n  Reached descent point ({dist_dp:.1f}m)")
            break
        if time.time() - t0 > 180:
            print(f"\n  DP timeout ({dist_dp:.1f}m) -- descending anyway")
            break
        time.sleep(LOOP_DT)

    # Stabilize 4 seconds
    print("  Stabilising 4s ...")
    t_hold = time.time()
    while time.time() - t_hold < 4.0:
        lat, lon, alt, yaw = get_state(vehicle)
        n_err, e_err = ned_offset_m(lat, lon, *DESCENT_POINT)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)
        alt_err = TARGET_ALT - alt
        alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
        thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                        RC_MIN, RC_MAX))
        ch2 = RC_MID - int(clamp(CRUISE_NAV_KP * fwd, -80, 80))
        ch1 = RC_MID + int(clamp(CRUISE_NAV_KP * rgt, -80, 80))
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)
        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)
        time.sleep(LOOP_DT)

    # STEP 3: Fast descent
    print(f"  DESCENDING -- Thr={FAST_DESCENT_THR}, flare at {FLARE_ALT}m")
    t0 = time.time()
    prev_alt = alt
    vz_filtered = 0.0
    t_prev = time.time()

    while True:
        t_now = time.time()
        dt = max(t_now - t_prev, 0.01)
        t_prev = t_now

        lat, lon, alt, yaw = get_state(vehicle)
        dist_b = haversine_m(lat, lon, *POINT_B)

        # Minimal horizontal correction toward B
        n_err, e_err = ned_offset_m(lat, lon, *POINT_B)
        fwd, rgt = ned_to_body(n_err, e_err, yaw)
        ch2 = RC_MID - int(clamp(FINE_NAV_KP * fwd * 0.3, -50, 50))
        ch1 = RC_MID + int(clamp(FINE_NAV_KP * rgt * 0.3, -50, 50))

        # Vertical speed
        vz_raw = (alt - prev_alt) / dt
        prev_alt = alt
        vz_filtered = 0.7 * vz_filtered + 0.3 * vz_raw

        # Throttle: fast drop above flare, gentle below
        if alt > FLARE_ALT:
            land_thr = FAST_DESCENT_THR
        else:
            land_thr = FLARE_THR

        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)
        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=land_thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [LAND] Alt:{alt:6.1f}m Dist-B:{dist_b:5.1f}m "
            f"Thr:{land_thr} Vz:{vz_filtered:+.1f}m/s Yaw:{yaw:.1f}   "
        )
        sys.stdout.flush()

        if alt < 0.3 or not vehicle.armed:
            print(f"\n  Touchdown! dist from B = {dist_b:.2f} m")
            break
        if time.time() - t0 > TIMEOUT_LAND:
            print(f"\n  Landing timeout (alt={alt:.1f}m)")
            break
        time.sleep(LOOP_DT)


# ==================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArduCopter STABILIZE A->B via RC Override")
    parser.add_argument("--connect", default="udp:127.0.0.1:14550",
                        help="MAVLink connection string")
    args = parser.parse_args()
    main(args.connect)
