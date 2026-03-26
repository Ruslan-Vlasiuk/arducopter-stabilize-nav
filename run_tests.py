#!/usr/bin/env python3
"""
Automated test runner for flight.py
Runs 5 consecutive SITL flights, collects results, prints statistics.
"""

import subprocess
import re
import time
import os
import sys
import signal
from datetime import datetime

# ── Configuration ──────────────────────────────────────────────────
NUM_FLIGHTS = 5
PAUSE_BETWEEN = 10  # seconds between flights

SITL_BIN = os.path.expanduser("~/ardupilot/build/sitl/bin/arducopter")
SITL_DEFAULTS = os.path.expanduser(
    "~/ardupilot/Tools/autotest/default_params/copter.parm"
)
SITL_HOME = "50.450739,30.461242,160,0"
SITL_INSTANCE = 1  # -I1 -> port 5770

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FLIGHT_SCRIPT = os.path.join(SCRIPT_DIR, "flight.py")
PYTHON = os.path.join(SCRIPT_DIR, "venv", "bin", "python3")
MAVPROXY_BIN = os.path.join(SCRIPT_DIR, "venv", "bin", "mavproxy.py")
SITL_EXTRA_PARM = os.path.join(SCRIPT_DIR, "sitl_extra.parm")

SITL_TCP_PORT = 5760 + SITL_INSTANCE * 10  # 5770 (SERIAL0 — flight.py)
SITL_TCP_PORT2 = SITL_TCP_PORT + 2          # 5772 (SERIAL1 — MAVProxy)
FG_VIEW_PORT = 5503 + SITL_INSTANCE * 10    # 5513 (FlightGear FDM)
CONNECT_STR = f"tcp:127.0.0.1:{SITL_TCP_PORT}"

FGFS_BIN = "/Applications/FlightGear.app/Contents/MacOS/FlightGear"
FG_AIRCRAFT_DIR = os.path.expanduser("~/ardupilot/Tools/autotest/aircraft")

RESULTS_FILE = os.path.join(SCRIPT_DIR, "results.txt")

FLIGHT_TIMEOUT = 360  # max seconds per flight before killing
SITL_STARTUP_WAIT = 10  # seconds to wait for SITL to start
MAVPROXY_STARTUP_WAIT = 5  # seconds to wait for MAVProxy


# ── Helpers ────────────────────────────────────────────────────────
def kill_all():
    """Kill any running SITL and MAVProxy processes for our instance."""
    for pattern in [f"arducopter.*-I{SITL_INSTANCE}", "mavproxy"]:
        try:
            subprocess.run(
                ["pkill", "-f", pattern],
                capture_output=True, timeout=5
            )
        except Exception:
            pass
    time.sleep(2)


def start_sitl():
    """Launch SITL in background, return Popen handle."""
    cmd = [
        SITL_BIN,
        "--model", "+",
        "--speedup", "1",
        "--defaults", f"{SITL_DEFAULTS},{SITL_EXTRA_PARM}",
        "--home", SITL_HOME,
        "--enable-fgview",
        "--wipe",
        f"-I{SITL_INSTANCE}",
    ]
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    return proc



def start_mavproxy():
    """Launch MAVProxy in a new Terminal.app window on SERIAL1 (5772)."""
    mavproxy_cmd = (
        f"{MAVPROXY_BIN} "
        f"--master=tcp:127.0.0.1:{SITL_TCP_PORT2} "
        f"--console --map"
    )
    apple_script = f'''
    tell application "Terminal"
        activate
        do script "{mavproxy_cmd}"
    end tell
    '''
    subprocess.Popen(
        ["osascript", "-e", apple_script],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def start_flightgear():
    """Launch FlightGear for 3D drone visualization."""
    if not os.path.exists(FGFS_BIN):
        print("  WARNING: FlightGear not found, skipping 3D view")
        return
    cmd = [
        FGFS_BIN,
        f"--native-fdm=socket,in,10,,{FG_VIEW_PORT},udp",
        "--fdm=external",
        "--aircraft=arducopter",
        f"--fg-aircraft={FG_AIRCRAFT_DIR}",
        "--geometry=800x600",
        "--disable-hud-3d",
        "--disable-horizon-effect",
        "--timeofday=noon",
        "--disable-sound",
        "--disable-fullscreen",
        "--disable-random-objects",
        "--disable-ai-models",
        "--fog-disable",
        "--disable-specular-highlight",
        "--disable-anti-alias-hud",
        "--wind=0@0",
    ]
    subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def run_flight(flight_num):
    """
    Run a single flight. Returns dict with results or None on failure.
    Streams output in real-time.
    """
    print(f"\n{'─' * 65}")
    print(f"  FLIGHT {flight_num}/{NUM_FLIGHTS}")
    print(f"{'─' * 65}")

    # Kill old SITL, start fresh
    kill_all()
    print(f"  Starting SITL (instance {SITL_INSTANCE}, TCP {SITL_TCP_PORT})...")
    sitl_proc = start_sitl()
    print(f"  Waiting {SITL_STARTUP_WAIT}s for SITL startup...")
    time.sleep(SITL_STARTUP_WAIT)

    # Check SITL is alive
    if sitl_proc.poll() is not None:
        print("  ERROR: SITL failed to start")
        return None

    # Start MAVProxy on SERIAL1 (port 5772) for 2D map
    print(f"  Starting MAVProxy on TCP:{SITL_TCP_PORT2} (console + map)...")
    start_mavproxy()
    time.sleep(MAVPROXY_STARTUP_WAIT)

    # Run flight.py on SERIAL0 (port 5770)
    cmd = [PYTHON, "-u", FLIGHT_SCRIPT, "--connect", CONNECT_STR]
    print(f"  Running: {' '.join(cmd)}\n")

    output_lines = []
    result = {
        "flight_num": flight_num,
        "distance": None,
        "time": None,
        "yaw": None,
        "status": "FAILED",
    }

    try:
        flight_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            bufsize=1,
            universal_newlines=True,
        )

        start_time = time.time()

        for line in flight_proc.stdout:
            line = line.rstrip("\n")
            output_lines.append(line)

            # Real-time output
            sys.stdout.write(line + "\n")
            sys.stdout.flush()

            # Parse results from output
            m = re.search(r"Distance from B:\s*([\d.]+)\s*m", line)
            if m:
                result["distance"] = float(m.group(1))

            m = re.search(r"Flight time\s*:\s*(\d+)\s*s", line)
            if m:
                result["time"] = int(m.group(1))

            m = re.search(r"Constant yaw\s*:\s*([\d.]+)\s*deg", line)
            if m:
                result["yaw"] = float(m.group(1))

            if "Touchdown!" in line:
                result["status"] = "OK"

            if "FLIGHT COMPLETE" in line:
                result["status"] = "OK"

            # Timeout check
            if time.time() - start_time > FLIGHT_TIMEOUT:
                print(f"\n  TIMEOUT: Flight exceeded {FLIGHT_TIMEOUT}s, killing...")
                flight_proc.kill()
                break

        flight_proc.wait(timeout=30)

    except Exception as e:
        print(f"\n  ERROR: {e}")
        result["status"] = "FAILED"

    finally:
        # Kill SITL process group
        try:
            os.killpg(os.getpgid(sitl_proc.pid), signal.SIGTERM)
            sitl_proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(sitl_proc.pid), signal.SIGKILL)
            except Exception:
                pass
        kill_all()

    # Print flight result summary
    if result["distance"] is not None:
        print(f"\n  Result: {result['distance']:.2f}m from B, "
              f"{result['time']}s, status={result['status']}")
    else:
        print(f"\n  Result: FAILED (no distance parsed)")
        result["status"] = "FAILED"

    return result


def print_statistics(results):
    """Print formatted statistics table with box-drawing characters."""
    ok_results = [r for r in results if r and r["status"] == "OK" and r["distance"] is not None]

    print("\n")
    print("╔══════════════════════════════════════════════════════════════════╗")
    print("║                    TEST RESULTS SUMMARY                        ║")
    print("╠═══════╦══════════════════╦══════════════╦═══════════╦══════════╣")
    print("║  Run  ║  Distance from B ║  Flight time ║    Yaw    ║  Status  ║")
    print("╠═══════╬══════════════════╬══════════════╬═══════════╬══════════╣")

    for r in results:
        if r and r["distance"] is not None:
            dist_str = f"{r['distance']:>8.2f} m"
            time_str = f"{r['time']:>6d} s" if r["time"] else "     N/A"
            yaw_str = f"{r['yaw']:>7.1f}°" if r["yaw"] else "     N/A"
            status_str = f"{r['status']:>6s}"
        else:
            dist_str = "     N/A   "
            time_str = "     N/A"
            yaw_str = "     N/A"
            status_str = "FAILED"
        print(f"║  {r['flight_num'] if r else '?':>3}  ║  {dist_str:>14s}  ║  {time_str:>10s}  ║ {yaw_str:>8s}  ║ {status_str:>6s}   ║")

    print("╠═══════╬══════════════════╬══════════════╬═══════════╬══════════╣")

    if ok_results:
        distances = [r["distance"] for r in ok_results]
        times = [r["time"] for r in ok_results if r["time"]]
        yaws = [r["yaw"] for r in ok_results if r["yaw"]]

        avg_dist = sum(distances) / len(distances)
        best_dist = min(distances)
        worst_dist = max(distances)
        avg_time = sum(times) / len(times) if times else 0
        avg_yaw = sum(yaws) / len(yaws) if yaws else 0

        print(f"║  AVG  ║  {avg_dist:>8.2f} m      ║  {avg_time:>6.0f} s    ║ {avg_yaw:>7.1f}°  ║  {len(ok_results)}/{len(results)} OK  ║")
        print(f"║  BEST ║  {best_dist:>8.2f} m      ║              ║           ║          ║")
        print(f"║  WRST ║  {worst_dist:>8.2f} m      ║              ║           ║          ║")
    else:
        print("║       ║   ALL FLIGHTS FAILED — no statistics available      ║")

    print("╚═══════╩══════════════════╩══════════════╩═══════════╩══════════╝")


def save_results(results):
    """Save full results to results.txt."""
    ok_results = [r for r in results if r and r["status"] == "OK" and r["distance"] is not None]

    with open(RESULTS_FILE, "w") as f:
        f.write(f"Flight Test Results — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"{'=' * 65}\n\n")
        f.write(f"SITL: {SITL_BIN}\n")
        f.write(f"Home: {SITL_HOME}\n")
        f.write(f"Connection: {CONNECT_STR}\n")
        f.write(f"Flights: {NUM_FLIGHTS}\n\n")

        f.write(f"{'Run':<6} {'Distance':>10} {'Time':>8} {'Yaw':>8} {'Status':>8}\n")
        f.write(f"{'-'*6} {'-'*10} {'-'*8} {'-'*8} {'-'*8}\n")

        for r in results:
            if r and r["distance"] is not None:
                f.write(
                    f"{r['flight_num']:<6} "
                    f"{r['distance']:>9.2f}m "
                    f"{r['time'] if r['time'] else 'N/A':>7}s "
                    f"{r['yaw'] if r['yaw'] else 'N/A':>7}° "
                    f"{r['status']:>8}\n"
                )
            else:
                num = r["flight_num"] if r else "?"
                f.write(f"{num:<6} {'N/A':>10} {'N/A':>8} {'N/A':>8} {'FAILED':>8}\n")

        f.write(f"\n{'=' * 65}\n")

        if ok_results:
            distances = [r["distance"] for r in ok_results]
            times = [r["time"] for r in ok_results if r["time"]]
            yaws = [r["yaw"] for r in ok_results if r["yaw"]]

            f.write(f"Average distance : {sum(distances)/len(distances):.2f} m\n")
            f.write(f"Best distance    : {min(distances):.2f} m\n")
            f.write(f"Worst distance   : {max(distances):.2f} m\n")
            if times:
                f.write(f"Average time     : {sum(times)/len(times):.0f} s\n")
            if yaws:
                f.write(f"Average yaw      : {sum(yaws)/len(yaws):.1f} deg\n")
            f.write(f"Success rate     : {len(ok_results)}/{len(results)}\n")
        else:
            f.write("ALL FLIGHTS FAILED\n")

    print(f"\n  Results saved to: {RESULTS_FILE}")


# ── Main ───────────────────────────────────────────────────────────
def main():
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║           AUTOMATED FLIGHT TEST RUNNER                     ║")
    print(f"║           {datetime.now().strftime('%Y-%m-%d %H:%M:%S'):<47s}  ║")
    print(f"║           Flights: {NUM_FLIGHTS}   Pause: {PAUSE_BETWEEN}s between runs        ║")
    print("╚══════════════════════════════════════════════════════════════╝")

    # Launch FlightGear once for all flights (takes time to load terrain)
    print(f"\n  Starting FlightGear on UDP:{FG_VIEW_PORT} (3D view)...")
    start_flightgear()
    print(f"  FlightGear loading... it will connect when SITL starts.\n")

    results = []

    for i in range(1, NUM_FLIGHTS + 1):
        result = run_flight(i)
        results.append(result)

        if i < NUM_FLIGHTS:
            print(f"\n  Pausing {PAUSE_BETWEEN}s before next flight...")
            time.sleep(PAUSE_BETWEEN)

    # Statistics
    print_statistics(results)
    save_results(results)

    # Exit code: 0 if all OK, 1 if any failed
    ok_count = sum(1 for r in results if r and r["status"] == "OK")
    sys.exit(0 if ok_count == NUM_FLIGHTS else 1)


if __name__ == "__main__":
    main()
