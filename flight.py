#!/usr/bin/env python3
"""
ArduCopter STABILIZE Flight -- RC Override Navigation (Cascaded Velocity Control)

A (start) : 50.450739, 30.461242
B (target): 50.443326, 30.448078
Altitude  : 200 m

Phases:
  1. Takeoff in STABILIZE -> 200 m
  2. Cruise: PI altitude hold at 200m, cascaded velocity nav toward B
  3. Glide: 18-deg slope to B, steep 35-deg in final 20m
  4. Flare (alt < 2m): reduce descent rate, maintain approach velocity

Navigation: cascaded controller
  Outer loop: position P -> desired velocity
  Inner loop: velocity PI -> RC pitch/roll
  Low-pass EMA filter on all velocities
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

# -- Altitude PI (used during takeoff and level cruise)
HOVER_THROTTLE = 1500
ALT_KP = 3.5
ALT_KI = 0.15
ALT_I_MAX = 120.0
ALT_MAX_DELTA = 350

# -- Cascaded navigation controller
#    Outer loop: position P -> desired velocity (m/s)
POS_KP = 0.5             # m/s per m of error
MAX_VEL_CRUISE = 12.0    # max desired speed during cruise
MAX_VEL_GLIDE = 8.0      # max during high-alt glide
MAX_VEL_LOW = 3.0        # max during low-alt glide (< 10m)

#    Inner loop: velocity PI -> RC delta
VEL_KP = 55.0            # RC units per m/s of velocity error
VEL_KI = 8.0             # RC units per m*s of integrated velocity error
VEL_I_MAX = 200.0        # integral clamp
NAV_MAX_CTRL = 380       # RC output clamp

# -- Velocity EMA filter
VEL_EMA_ALPHA = 0.25     # low-pass filter on body-frame velocities

# -- Yaw P
YAW_KP = 4.0
YAW_DEADZONE = 0.5

# -- Glide parameters
GLIDE_ANGLE_DEG = 18.0   # steeper glide = start closer to B, faster flight
GLIDE_START_DIST = TARGET_ALT / math.tan(math.radians(GLIDE_ANGLE_DEG))  # ~615m

# -- Descent rate controller (PI + feedforward)
DESCENT_KP = 50.0
DESCENT_KI = 3.0
DESCENT_FF = 65.0   # feedforward: thr = HOVER - FF * |target_vz|

# -- Wind parameters for aim point compensation
WIND_SPD = 3.0
WIND_DIR_DEG = 30.0  # direction wind blows FROM

# -- Timeouts
LOOP_DT = 0.08
TIMEOUT_TAKEOFF = 180
TIMEOUT_FLIGHT = 900
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

def offset_point(lat, lon, bearing_deg_val, distance_m):
    """Return (lat2, lon2) offset from (lat,lon) by distance_m at bearing_deg."""
    brng = math.radians(bearing_deg_val)
    lat2 = lat + (distance_m * math.cos(brng)) / 111320.0
    lon2 = lon + (distance_m * math.sin(brng)) / (111320.0 * math.cos(math.radians(lat)))
    return lat2, lon2


# ==================================================================
#  DRONEKIT / MAVLINK HELPERS
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
        print("    ... waiting for arm", end="\r")
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
    """Returns (lat, lon, alt, yaw_deg, vx, vy, vz) where vx/vy/vz are NED m/s."""
    global _last_valid_lat, _last_valid_lon, _last_valid_alt
    loc = vehicle.location.global_relative_frame
    lat = loc.lat
    lon = loc.lon
    alt = loc.alt

    if lat is not None and lon is not None and abs(lat) > 1.0:
        _last_valid_lat = lat
        _last_valid_lon = lon
        _last_valid_alt = alt if alt is not None else _last_valid_alt
    else:
        lat = _last_valid_lat
        lon = _last_valid_lon
        alt = _last_valid_alt

    yaw_rad = vehicle.attitude.yaw
    yaw_d = (math.degrees(yaw_rad) + 360.0) % 360.0

    # NED velocity from EKF
    vel = vehicle.velocity
    vx = vel[0] if vel and vel[0] is not None else 0.0
    vy = vel[1] if vel and vel[1] is not None else 0.0
    vz = vel[2] if vel and vel[2] is not None else 0.0

    return (lat, lon, alt, yaw_d, vx, vy, vz)

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

    print("=" * 65)
    print("  DRONE STABILIZE FLIGHT -- RC Override (Glide approach)")
    print(f"  A -> B bearing  : {FLIGHT_YAW:.1f} deg (CONSTANT YAW)")
    print(f"  Route distance  : {DIST_TOTAL:.0f} m")
    print(f"  Target altitude : {TARGET_ALT:.0f} m")
    print(f"  Glide angle     : {GLIDE_ANGLE_DEG:.1f} deg")
    print(f"  Glide start dist: {GLIDE_START_DIST:.0f} m from B")
    print("=" * 65)

    print(f"\nConnecting to {connection_string} ...")
    vehicle = connect(connection_string, wait_ready=True, timeout=60)
    print(f"Connected  |  FW: {vehicle.version}  |  Mode: {vehicle.mode.name}")

    try:
        vehicle.parameters['RC_OVERRIDE_TIME'] = 3.0
    except Exception:
        pass

    # Wait for GPS/EKF
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
        print("\n  WARNING: GPS/EKF timeout")

    lat0, lon0, alt0, _, _, _, _ = get_state(vehicle)
    home_dist = haversine_m(lat0, lon0, *POINT_A)
    print(f"  Current position: {lat0:.6f}, {lon0:.6f}")
    print(f"  Distance from Point A: {home_dist:.1f} m")

    # Wind parameters
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

    flight_start_time = time.time()
    try:
        _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL)
    except KeyboardInterrupt:
        print("\n  INTERRUPTED by user")
    except Exception as e:
        print(f"\n  ERROR: {e}")
        import traceback
        traceback.print_exc()
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

        lat, lon, alt, _, _, _, _ = get_state(vehicle)
        final_dist = haversine_m(lat, lon, *POINT_B)

        print("\n" + "=" * 65)
        print("  FLIGHT COMPLETE")
        print(f"  Final position : {lat:.6f}, {lon:.6f}")
        print(f"  Final altitude : {alt:.2f} m")
        print(f"  Distance from B: {final_dist:.2f} m")
        print(f"  Constant yaw   : {FLIGHT_YAW:.1f} deg")
        print(f"  Flight time    : {int(time.time() - flight_start_time)} s")
        print("=" * 65)

        vehicle.close()


def _run_flight(vehicle, FLIGHT_YAW, DIST_TOTAL):

    # ==============================================================
    # PHASE 1 -- TAKEOFF (STABILIZE, RC override)
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
        lat, lon, alt, yaw, _, _, _ = get_state(vehicle)
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
            f"\r  [TKOFF]  Alt {alt:6.1f}/{TARGET_ALT:.0f}m  "
            f"Thr {int(thr)}  Yaw {yaw:5.1f}->{FLIGHT_YAW:.1f}   "
        )
        sys.stdout.flush()

        if alt >= TARGET_ALT - 1.0:
            print(f"\n  Target altitude reached -- {alt:.1f} m")
            break

        if time.time() - t0 > TIMEOUT_TAKEOFF:
            print("\n  Takeoff timeout -- aborting")
            return

        time.sleep(LOOP_DT)

    # ==============================================================
    # PHASE 2 -- CRUISE + GLIDE to B (PID nav, fast descent)
    # ==============================================================
    # PHASE 2 -- CRUISE + GLIDE + FLARE to B (Cascaded Velocity Control)
    # ==============================================================
    print(f"\n{'-'*65}")
    print(f"  PHASE 2 -- CRUISE + GLIDE + FLARE -> Point B ({DIST_TOTAL:.0f}m)")
    print(f"  Glide starts at {GLIDE_START_DIST:.0f}m from B")
    print(f"  Cascaded nav: position P -> desired velocity -> velocity PI -> RC")
    print(f"  Constant YAW = {FLIGHT_YAW:.1f} deg")
    print(f"{'-'*65}")

    # Calculate wind-compensated aim point
    est_glide_time = GLIDE_START_DIST / 8.0
    wind_drift = WIND_SPD * est_glide_time
    aim_lat, aim_lon = offset_point(POINT_B[0], POINT_B[1], WIND_DIR_DEG, wind_drift * 0.15)
    aim_dist = haversine_m(aim_lat, aim_lon, *POINT_B)
    print(f"  Wind aim offset: {aim_dist:.0f}m upwind from B")

    alt_integral = 0.0
    vel_fwd_integral = 0.0
    vel_rgt_integral = 0.0
    vz_filtered = 0.0
    vz_integral = 0.0
    prev_alt = alt

    # EMA-filtered horizontal velocities (body frame)
    vfwd_ema = 0.0
    vrgt_ema = 0.0

    t_prev = time.time()
    t0 = time.time()
    gliding = False
    flaring = False

    while True:
        t_now = time.time()
        dt_actual = max(t_now - t_prev, 0.01)
        t_prev = t_now

        lat, lon, alt, yaw, vx_ned, vy_ned, vz_ned = get_state(vehicle)
        dist_b = haversine_m(lat, lon, *POINT_B)

        # Convert NED velocity to body frame and low-pass filter
        v_fwd_raw, v_rgt_raw = ned_to_body(vx_ned, vy_ned, yaw)
        vfwd_ema = (1 - VEL_EMA_ALPHA) * vfwd_ema + VEL_EMA_ALPHA * v_fwd_raw
        vrgt_ema = (1 - VEL_EMA_ALPHA) * vrgt_ema + VEL_EMA_ALPHA * v_rgt_raw
        hspeed = math.sqrt(vfwd_ema**2 + vrgt_ema**2)

        # Phase transitions
        if not gliding and dist_b <= GLIDE_START_DIST:
            gliding = True
            print(f"\n  === GLIDE START at dist={dist_b:.0f}m alt={alt:.0f}m ===")
            vz_integral = 0.0

        if gliding and not flaring and alt < 2.0 and dist_b < 10:
            flaring = True
            print(f"\n  === FLARE at dist={dist_b:.1f}m alt={alt:.1f}m hspd={hspeed:.1f} ===")

        # -- Navigation target with wind compensation
        if gliding and dist_b > 30:
            nav_target = (aim_lat, aim_lon)
        elif gliding and dist_b > 5:
            # Fixed upwind offset to counter residual wind drift
            nav_target = offset_point(POINT_B[0], POINT_B[1], WIND_DIR_DEG, 4.5)
        else:
            nav_target = POINT_B

        # Position error in body frame
        n_err, e_err = ned_offset_m(lat, lon, *nav_target)
        fwd_err, rgt_err = ned_to_body(n_err, e_err, yaw)

        # === CASCADED NAV CONTROLLER ===
        # Outer loop: position P -> desired velocity
        if gliding and alt < 10:
            max_vel = MAX_VEL_LOW
        elif gliding:
            max_vel = MAX_VEL_GLIDE
        else:
            max_vel = MAX_VEL_CRUISE

        desired_vel_fwd = clamp(POS_KP * fwd_err, -max_vel, max_vel)
        desired_vel_rgt = clamp(POS_KP * rgt_err, -max_vel, max_vel)

        # Inner loop: velocity PI -> RC delta
        vel_err_fwd = desired_vel_fwd - vfwd_ema
        vel_err_rgt = desired_vel_rgt - vrgt_ema

        vel_fwd_integral = clamp(vel_fwd_integral + vel_err_fwd * dt_actual, -VEL_I_MAX, VEL_I_MAX)
        vel_rgt_integral = clamp(vel_rgt_integral + vel_err_rgt * dt_actual, -VEL_I_MAX, VEL_I_MAX)

        pitch_delta = clamp(VEL_KP * vel_err_fwd + VEL_KI * vel_fwd_integral,
                            -NAV_MAX_CTRL, NAV_MAX_CTRL)
        roll_delta = clamp(VEL_KP * vel_err_rgt + VEL_KI * vel_rgt_integral,
                           -NAV_MAX_CTRL, NAV_MAX_CTRL)

        ch2 = RC_MID - int(pitch_delta)
        ch1 = RC_MID + int(roll_delta)
        ch4 = ch4_for_yaw(yaw, FLIGHT_YAW)

        # -- Altitude / descent control
        vz_raw = (alt - prev_alt) / dt_actual
        prev_alt = alt
        vz_filtered = 0.7 * vz_filtered + 0.3 * vz_raw

        if not gliding:
            # Level cruise -- PI altitude hold at TARGET_ALT
            alt_err = TARGET_ALT - alt
            alt_integral = clamp(alt_integral + alt_err * LOOP_DT, -ALT_I_MAX, ALT_I_MAX)
            thr = int(clamp(HOVER_THROTTLE + ALT_KP * alt_err + ALT_KI * alt_integral,
                            RC_MIN, RC_MAX))
            phase_str = "CRUISE"
        elif flaring:
            # FLARE: moderate descent over B, velocity controller holds position
            target_vz = -0.8
            vz_err = target_vz - vz_filtered
            vz_integral = clamp(vz_integral + vz_err * dt_actual, -50, 50)
            thr = int(clamp(
                HOVER_THROTTLE - DESCENT_FF * abs(target_vz) + DESCENT_KP * vz_err + DESCENT_KI * vz_integral,
                RC_MIN, RC_MAX
            ))
            phase_str = f"FLARE d={dist_b:4.1f}"
        else:
            # Glide -- target altitude follows glide slope
            if dist_b < 20:
                glide_tan = math.tan(math.radians(35.0))
            else:
                glide_tan = math.tan(math.radians(GLIDE_ANGLE_DEG))
            target_alt = max(dist_b * glide_tan, 0.0)

            alt_err_glide = target_alt - alt

            if alt > 10:
                target_vz = -3.0 + alt_err_glide * 0.15
            elif alt > 3:
                target_vz = -1.2
            else:
                target_vz = -0.6

            if alt > target_alt + 5:
                target_vz -= 1.5
            if alt < target_alt - 3:
                target_vz += 2.0

            target_vz = clamp(target_vz, -8.0, -0.3)

            vz_err = target_vz - vz_filtered
            vz_integral = clamp(vz_integral + vz_err * dt_actual, -50, 50)
            thr = int(clamp(
                HOVER_THROTTLE - DESCENT_FF * abs(target_vz) + DESCENT_KP * vz_err + DESCENT_KI * vz_integral,
                RC_MIN, RC_MAX
            ))
            phase_str = f"GLIDE t_alt={target_alt:5.1f}"

        rc_override(vehicle, roll=ch1, pitch=ch2, throttle=thr, yaw=ch4)

        sys.stdout.write(
            f"\r  [{phase_str}]  Dist {dist_b:6.1f}m  Alt {alt:5.1f}m  "
            f"Thr {thr}  Vz {vz_filtered:+5.1f}  Hspd {hspeed:4.1f}  "
        )
        sys.stdout.flush()

        # Touchdown detection
        if gliding and (alt < 0.2 or not vehicle.armed):
            print(f"\n  Touchdown!  dist from B = {dist_b:.2f} m  hspd = {hspeed:.2f} m/s")
            break

        if time.time() - t0 > TIMEOUT_FLIGHT:
            print(f"\n  Flight timeout (dist={dist_b:.1f}m alt={alt:.1f}m)")
            break

        time.sleep(LOOP_DT)


# ==================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ArduCopter STABILIZE flight A->B via RC Override (Glide)"
    )
    parser.add_argument(
        "--connect",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string"
    )
    args = parser.parse_args()
    main(args.connect)
