#!/usr/bin/env python3
"""Team 14 DAQ: guided menu. Anyone can run the rig from here.

    cd ~/daq
    python3 daq.py          # on the Pi, with the sensors
    python3 daq.py --sim    # on any computer, fake sensors (practice / demo)

Every option explains what it does and what a good result looks like.
"""
import csv
import glob
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.expanduser("~/runs")
SIM = "--sim" in sys.argv
PY = sys.executable
LINE = "=" * 64


def clear():
    print("\n" * 2)


def pause():
    input("\nPress Enter to go back to the menu...")


def run(args):
    """Run one of the tools in the same terminal; Ctrl+C returns to the menu."""
    try:
        subprocess.call([PY, os.path.join(HERE, args[0])] + args[1:])
    except KeyboardInterrupt:
        pass


def latest_run():
    runs = sorted(glob.glob(os.path.join(RUNS, "run_*")), key=os.path.getmtime)
    return runs[-1] if runs else None


def status():
    if SIM:
        return "SIMULATION MODE: fake sensors, nothing is real data"
    parts = []
    try:
        from smbus2 import SMBus
        with SMBus(1) as bus:
            for name, addr in (("IMU", 0x68), ("BARO", 0x77)):
                try:
                    bus.read_byte(addr)
                    parts.append(f"{name} ok")
                except OSError:
                    parts.append(f"{name} MISSING")
    except Exception:
        parts.append("I2C off (run setup_pi.sh)")
    parts.append("GPS port ok" if os.path.exists("/dev/serial0") else "GPS port MISSING")
    parts.append("PPS ok" if os.path.exists("/dev/pps0") else "PPS MISSING")
    parts.append("LoRa ok" if glob.glob("/dev/ttyACM*") else "LoRa not plugged (optional)")
    return " | ".join(parts)


# ----------------------------------------------------------------- options
def explain(title, what, steps, good):
    print(f"\n{LINE}\n{title}\n{LINE}")
    print(f"WHAT IT DOES: {what}")
    print("WHAT YOU DO:")
    for s in steps:
        print(f"  - {s}")
    print(f"GOOD RESULT : {good}")
    print()


def check_imu():
    explain("1) CHECK IMU (MPU6050)",
            "Talks to the motion sensor and prints acceleration and rotation 5 times per second.",
            ["Leave the rig flat and still for 5 seconds.",
             "Then tilt it slowly in different directions, then twist it flat on the table."],
            "ID 0x68 PASS; flat and still: |a| about 9.8 [PASS], rotation near 0; "
            "tilting moves the 9.8 between ax/ay/az.")
    if SIM:
        print("(Simulation: sensor checks need the real hardware.)")
        return pause()
    input("Press Enter to start...")
    run(["sensor_check.py", "imu"])
    pause()


def check_baro():
    explain("2) CHECK BAROMETER (BMP180)",
            "Reads air pressure and converts it to altitude, 5 times per second.",
            ["Leave it on the desk for a few readings.",
             "Lift the rig about 1 m, hold it, then put it back down."],
            "ID 0x55 PASS; pressure 97,000-103,000 Pa [PASS]; lifting 1 m drops pressure ~12 Pa "
            "and raises altitude ~1 m; it returns when put back.")
    if SIM:
        print("(Simulation: sensor checks need the real hardware.)")
        return pause()
    input("Press Enter to start...")
    run(["sensor_check.py", "baro"])
    pause()


def check_gps():
    explain("3) CHECK GPS + TIMING PULSE",
            "Listens to the GPS, then waits for a position fix. Afterwards shows the timing-pulse test.",
            ["Do this OUTDOORS or right at a window, antenna facing the sky.",
             "Wait 1-5 minutes. The HAT's Fix LED blinks every 1 s while searching, every ~15 s with a fix."],
            "Valid NMEA lines [PASS]; then 'FIX ... sats >= 5 [PASS]' with a position that matches your "
            "phone's map. Stop the live readout with Ctrl+C.")
    if SIM:
        print("(Simulation: sensor checks need the real hardware.)")
        return pause()
    input("Press Enter to start...")
    run(["sensor_check.py", "gps"])
    print("\nTiming pulse test (one line per second = good). Ctrl+C to stop.")
    try:
        subprocess.call(["sudo", "ppstest", "/dev/pps0"])
    except KeyboardInterrupt:
        pass
    print("\nClock lock (look for a line starting with #* PPS):")
    subprocess.call(["chronyc", "sources"])
    pause()


def check_lora():
    explain("8) CHECK LoRa NODE (Meshtastic, 915 MHz)",
            "Talks to the XIAO LoRa node over USB, shows its region, then listens 60 s and prints "
            "every packet heard (who, signal strength RSSI, SNR). It never sends a message.",
            ["Screw the antenna on BEFORE powering the node (no antenna = can damage the radio).",
             "Plug the node into any Pi USB port with the USB-C cable.",
             "Wait 60 s. Ctrl+C stops early."],
            "Region US. Packets listed = good. Nothing heard = node works, just nobody transmitting nearby.")
    run(["lora_check.py"] + (["--sim"] if SIM else []))
    pause()


def lte_scan():
    explain("9) LTE TOWER SCAN (LimeSDR Mini 2.0, Band 5 869-894 MHz, RECEIVE ONLY)",
            "Sweeps the 850 MHz LTE band, finds carriers, decodes the ID (PCI) of every tower it hears.",
            ["Antenna on the RX port (NOT TX). LimeSDR in a BLUE USB 3.0 port.",
             "Outdoors or at a window. Type where you are standing as the label (e.g. near_C5025).",
             "Wait ~3 min. If it says CLIPPING, run again with a lower gain."],
            "A table of PCIs. The strongest N_ID1 next to a tower = that tower's ID -> write it in lte_towers.csv.")
    label = input("Where are you standing? (label, e.g. near_C5025): ").strip().replace(" ", "_")
    gain = input("Gain in dB [Enter = 40]: ").strip() or "40"
    run(["lte_scan.py", "--label", label, "--gain", gain] + (["--sim"] if SIM else []))
    pause()


def window_mean(rows, col, t0, t1):
    v = [float(r[col]) for r in rows if t0 <= float(r["time_s"]) <= t1]
    return sum(v) / len(v) if v else float("nan")


def axis_check():
    explain("4) AXIS CHECK (60 s, guided)",
            "Records 60 s while you tilt the rig as told, then confirms the IMU directions are right "
            "(x forward, y right, z down).",
            ["Hold or place the rig flat, X arrow on the MPU6050 pointing forward.",
             "Follow the countdown: FLAT, FRONT UP, FLAT, RIGHT SIDE DOWN, FLAT."],
            "Three PASS lines at the end. If any says CHECK, see the PDF section 9 (one-line fix).")
    input("Press Enter to start the 60 s recording...")
    args = [PY, os.path.join(HERE, "daq_logger.py"), "--name", "axischeck", "--duration", "60",
            "--out", RUNS] + (["--sim"] if SIM else [])
    p = subprocess.Popen(args, stdout=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
    plan = [(0, "Keep it FLAT and still"), (10, "Tilt the FRONT UP (~30 degrees) and hold"),
            (20, "Back to FLAT"), (30, "Tilt the RIGHT SIDE DOWN (~30 degrees) and hold"),
            (40, "Back to FLAT and hold until the end")]
    t0 = time.time()
    for k, (t, msg) in enumerate(plan):
        while time.time() - t0 < t:
            time.sleep(0.1)
        end = plan[k + 1][0] if k + 1 < len(plan) else 60
        print(f"  [{t:2d}-{end:2d} s]  {msg}")
    p.wait()
    run_dir = latest_run()
    with open(os.path.join(run_dir, "imu.csv")) as f:
        rows = list(csv.DictReader(f))
    az_flat = window_mean(rows, "az_mps2", 3, 9)
    ax_up = window_mean(rows, "ax_mps2", 13, 18)
    ay_right = window_mean(rows, "ay_mps2", 33, 38)
    res = [("Flat: az about -9.8", az_flat < -8.0, f"az = {az_flat:.2f}"),
           ("Front up: ax positive", ax_up > 2.0, f"ax = {ax_up:.2f}"),
           ("Right side down: ay positive", ay_right > 2.0, f"ay = {ay_right:.2f}")]
    print()
    for name, ok, val in res:
        print(f"  {'PASS ' if ok else 'CHECK'}  {name:32s} ({val} m/s2)")
    if not all(ok for _, ok, _ in res):
        print("\n  CHECK = the IMU is mounted differently than assumed, or you didn't tilt at that moment.\n"
              "  Repeat once; if it still fails, change AXIS_MAP in daq_logger.py (PDF section 9).")
    pause()


def record():
    print(f"\n{LINE}\n5) RECORD A RUN\n{LINE}")
    print("""BEFORE YOU START
  - Outdoors at your marked start spot, GPS Fix LED blinking slowly (every ~15 s).
  - Rig level, MPU6050 X arrow pointing forward, antenna on top.
  - Powered from the power bank if you are walking.
WHILE RECORDING
  - Stand completely STILL for 2 minutes, then type  static_end  and press Enter.
  - Walk straight ~20 m first (gives the starting heading).
  - Type a label + Enter at key moments: tunnel_in, tunnel_out, stairs_up, stairs_end, stop, go
  - To finish: stand still 30 s at the end, then type  q  and press Enter.""")
    name = input("\nName for this run (e.g. static, loop, tunnel, stairs): ").strip() or "run"
    name = re.sub(r"[^A-Za-z0-9_-]", "_", name)
    if input(f"Start recording '{name}' now? (y/n): ").strip().lower() != "y":
        return
    args = ["daq_logger.py", "--name", name, "--out", RUNS] + (["--sim"] if SIM else [])
    if input("Record LTE with the LimeSDR too? (y/n): ").strip().lower() == "y":
        fr = input("  Carriers in MHz [Enter = 875.2]: ").strip() or "875.2"
        gn = input("  Gain [Enter = 30]: ").strip() or "30"
        args += ["--lte", fr, "--lte-gain", gn]
    run(args)
    run_dir = latest_run()
    print(f"\nChecking the recording...\n")
    run(["check_run.py", run_dir])
    if input("\nAdd route notes / lever arm to the README now? (y/n): ").strip().lower() == "y":
        subprocess.call(["nano", os.path.join(run_dir, "README.txt")])
    pause()


def check_last():
    run_dir = latest_run()
    print(f"\n{LINE}\n6) CHECK THE LAST RUN\n{LINE}")
    if not run_dir:
        print("No recordings yet.")
        return pause()
    print("WHAT IT DOES: rates, gaps, gravity check at the start, GPS fixes, events; saves quicklook.png.")
    print("GOOD RESULT : the last line says 'ready to send'.\n")
    run(["check_run.py", run_dir])
    pause()


def copy_help():
    user = os.environ.get("USER", "YOURUSER")
    host = socket.gethostname()
    print(f"\n{LINE}\n7) COPY RUNS TO THE LAPTOP (instructions)\n{LINE}")
    runs = sorted(glob.glob(os.path.join(RUNS, "run_*")))
    print("Recordings on this Pi:")
    for r in runs[-10:]:
        print("  ", os.path.basename(r))
    last = os.path.basename(runs[-1]) if runs else "run_..."
    print(f"""
On the LAPTOP (open a NEW PowerShell / Terminal window, NOT this SSH one), type:

  scp -r {user}@{host}.local:~/runs/{last} .        (just the last run)
  scp -r {user}@{host}.local:~/runs .               (all runs)

Then: zip each run folder, upload to Teams > Files > data > real, and post the
message from the PDF (section 12).""")
    pause()


def shutdown():
    if SIM:
        print("(Simulation: not shutting anything down.)")
        return pause()
    if input("Shut the Pi down now? Wait for the green light to stop before unplugging. (y/n): ").lower() == "y":
        subprocess.call(["sudo", "shutdown", "now"])
        sys.exit(0)


MENU = [("1", "Check IMU (motion sensor)", check_imu),
        ("2", "Check barometer (altitude)", check_baro),
        ("3", "Check GPS + timing pulse (outdoors)", check_gps),
        ("4", "Axis check, 60 s guided (once per day)", axis_check),
        ("5", "RECORD A RUN", record),
        ("6", "Check the last run (ready to send?)", check_last),
        ("7", "How to copy runs to the laptop", copy_help),
        ("8", "Check LoRa node (optional, listens 60 s)", lambda: check_lora()),
        ("9", "LTE tower scan (LimeSDR, ~3 min)", lambda: lte_scan()),
        ("s", "Shut down the Pi safely", shutdown)]


def main():
    os.makedirs(RUNS, exist_ok=True)
    while True:
        clear()
        print(LINE)
        print("  TEAM 14 DAQ RIG  (IMU + barometer + GPS + LoRa -> CSV files)")
        print(f"  Status: {status()}")
        print(LINE)
        for key, label, _ in MENU:
            print(f"   {key})  {label}")
        print("   q)  Quit (leaves the Pi running)")
        print("\n  First time today: 1 -> 2 -> 3 -> 4.  Then 5 for every run, 6 after each run.")
        choice = input("\nChoose an option and press Enter: ").strip().lower()
        if choice == "q":
            break
        for key, _, fn in MENU:
            if choice == key:
                try:
                    fn()
                except KeyboardInterrupt:
                    pass
                except Exception as e:
                    print(f"\nSomething went wrong: {e}\nSee the PDF troubleshooting table (section 13).")
                    pause()
                break


if __name__ == "__main__":
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        print("\nbye")
