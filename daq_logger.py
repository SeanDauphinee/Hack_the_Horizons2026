#!/usr/bin/env python3
"""DAQ logger for the Team 14 rig: MPU6050 (IMU) + BMP180 (baro) + Adafruit GPS HAT.

Writes one folder per run:  ~/runs/run_YYYYMMDD_HHMMSS/
    imu.csv    ~100 Hz  time_s,ax_mps2,ay_mps2,az_mps2,gx_radps,gy_radps,gz_radps,temp_c
    baro.csv   ~30 Hz   time_s,pressure_pa,temp_c,alt_isa_m
    gnss.csv   1 Hz     time_s,utc_hhmmss,lat_deg,lon_deg,height_m,fix_quality,num_sats,
                        hdop,speed_mps,course_deg,mean_cn0_dbhz
    lora.csv   per packet  time_s,rx_time_unix,from_id,portnum,rssi_dbm,snr_db,hop_start,hop_limit,
                        sender_lat_deg,sender_lon_deg,sender_alt_m,pos_source
                        (only if the Meshtastic LoRa node is plugged in by USB; optional)
    events.csv          time_s,event
    combined.csv        ALL sensors in one file at the IMU rate, Meas_Bus layout (merge_run.py)
    nmea.log            every raw GPS sentence:  time_s,<sentence>
    README.txt          run metadata (start time, axis mapping, rates, your notes)

time_s = seconds since logger start, from the Pi system clock (disciplined by chrony + GPS PPS).
The absolute start time (UTC and Unix epoch) is written in README.txt.

Usage on the Pi:
    python3 daq_logger.py                      # record until you type q
    python3 daq_logger.py --name walk_tunnel   # adds a label to the folder name
    python3 daq_logger.py --sim --duration 30  # no hardware: fake sensors (test anywhere)
While recording, type an event label and press Enter (e.g. tunnel_in), or q + Enter to stop.
"""
import argparse
import math
import os
import queue
import random
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

G = 9.80665
MPU_ADDR, BMP_ADDR = 0x68, 0x77
IMU_RATE_HZ = 100
BMP_OSS = 3                       # BMP180 oversampling (3 = ultra high resolution, 25.5 ms)
TEMP_EVERY_S = 1.0                # BMP180 temperature refresh period
GPS_PORT, GPS_BAUD = "/dev/serial0", 9600
LORA_PORTS = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0")   # XIAO ESP32S3 shows up as ttyACM0

# Chip axes -> body axes (x forward, y right, z down).
# GY-521 mounted FLAT, components UP, printed X arrow pointing FORWARD:
#   chip x = forward, chip y = left, chip z = up  ->  body = (x, -y, -z)
# Change the signs/order here if you mount it differently, then re-check:
#   flat & still -> az ~ -9.81 ; nose tilted up -> ax goes positive ; right side down -> ay positive
AXIS_MAP = ((0, +1), (1, -1), (2, -1))   # (chip axis index, sign) for body x, y, z


# ======================================================================= helpers
def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def to_body(v):
    return tuple(sign * v[idx] for idx, sign in AXIS_MAP)


def isa_alt(p):
    return 44330.8 * (1.0 - (p / 101325.0) ** 0.190263)


def nmea_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, cks = line[1:].split("*", 1)
    c = 0
    for ch in body:
        c ^= ord(ch)
    try:
        return c == int(cks[:2], 16)
    except ValueError:
        return False


def nmea_deg(val, hemi, deg_len):
    if not val:
        return None
    d = float(val[:deg_len]) + float(val[deg_len:]) / 60.0
    return -d if hemi in ("S", "W") else d


# ======================================================================= BMP180 math
def bmp_calibration(raw22):
    names = ["AC1", "AC2", "AC3", "AC4", "AC5", "AC6", "B1", "B2", "MB", "MC", "MD"]
    cal = {}
    for i, n in enumerate(names):
        v = (raw22[2 * i] << 8) | raw22[2 * i + 1]
        if n not in ("AC4", "AC5", "AC6") and v > 32767:
            v -= 65536
        cal[n] = v
    return cal


def _div(a, b):
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def bmp_b5(cal, ut):
    x1 = ((ut - cal["AC6"]) * cal["AC5"]) >> 15
    x2 = _div(cal["MC"] << 11, x1 + cal["MD"])
    return x1 + x2


def bmp_pressure(cal, b5, up, oss):
    """Bosch datasheet algorithm (verified against the datasheet example: 69964 Pa)."""
    b6 = b5 - 4000
    x1 = (cal["B2"] * ((b6 * b6) >> 12)) >> 11
    x2 = (cal["AC2"] * b6) >> 11
    b3 = _div(((cal["AC1"] * 4 + x1 + x2) << oss) + 2, 4)
    x1 = (cal["AC3"] * b6) >> 13
    x2 = (cal["B1"] * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (cal["AC4"] * ((x3 + 32768) & 0xFFFFFFFF)) >> 15
    b7 = ((up - b3) & 0xFFFFFFFF) * (50000 >> oss)
    p = (b7 * 2) // b4 if b7 < 0x80000000 else (b7 // b4) * 2
    x1 = ((p >> 8) * (p >> 8) * 3038) >> 16
    x2 = (-7357 * p) >> 16
    return p + ((x1 + x2 + 3791) >> 4)


# ======================================================================= hardware / sim
class RealI2C:
    def __init__(self):
        from smbus2 import SMBus
        self.bus = SMBus(1)
        if self.bus.read_byte_data(MPU_ADDR, 0x75) != 0x68:
            raise RuntimeError("MPU6050 not found at 0x68 (check SDA/SCL/VCC/GND)")
        if self.bus.read_byte_data(BMP_ADDR, 0xD0) != 0x55:
            raise RuntimeError("BMP180 not found at 0x77 (check its 4 wires)")
        w = self.bus.write_byte_data
        w(MPU_ADDR, 0x6B, 0x01)   # wake, PLL with gyro X clock
        w(MPU_ADDR, 0x1A, 0x03)   # DLPF ~44 Hz
        w(MPU_ADDR, 0x19, 0x09)   # sample rate 1 kHz/(1+9) = 100 Hz
        w(MPU_ADDR, 0x1B, 0x00)   # gyro  +/-250 deg/s
        w(MPU_ADDR, 0x1C, 0x00)   # accel +/-2 g
        time.sleep(0.1)
        self.cal = bmp_calibration(self.bus.read_i2c_block_data(BMP_ADDR, 0xAA, 22))

    def read_mpu(self):
        d = self.bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
        acc = tuple(s16(d[i], d[i + 1]) / 16384.0 * G for i in (0, 2, 4))
        gyr = tuple(math.radians(s16(d[i], d[i + 1]) / 131.0) for i in (8, 10, 12))
        return acc, gyr, s16(d[6], d[7]) / 340.0 + 36.53

    def bmp_start_temp(self):
        self.bus.write_byte_data(BMP_ADDR, 0xF4, 0x2E)

    def bmp_read_temp(self):
        d = self.bus.read_i2c_block_data(BMP_ADDR, 0xF6, 2)
        return (d[0] << 8) | d[1]

    def bmp_start_press(self):
        self.bus.write_byte_data(BMP_ADDR, 0xF4, 0x34 + (BMP_OSS << 6))

    def bmp_read_press(self):
        d = self.bus.read_i2c_block_data(BMP_ADDR, 0xF6, 3)
        return ((d[0] << 16) | (d[1] << 8) | d[2]) >> (8 - BMP_OSS)


class SimI2C:
    """Fake sensors with realistic numbers, so the whole pipeline can be tested anywhere."""
    cal = dict(AC1=408, AC2=-72, AC3=-14383, AC4=32741, AC5=32757, AC6=23153,
               B1=6190, B2=4, MB=-32768, MC=-8711, MD=2868)

    def read_mpu(self):
        n = random.gauss
        acc_chip = (0.08 + n(0, 0.03), 0.05 + n(0, 0.03), G + 0.12 + n(0, 0.03))   # flat, z up
        gyr_chip = tuple(math.radians(b + n(0, 0.03)) for b in (0.6, -0.4, 0.9))
        return acc_chip, gyr_chip, 24.0 + n(0, 0.05)

    def bmp_start_temp(self):
        pass

    def bmp_read_temp(self):
        return 27898 + int(random.gauss(0, 2))

    def bmp_start_press(self):
        pass

    def bmp_read_press(self):
        return 271904 + int(random.gauss(0, 8))        # ~100.3 kPa with the sim calibration


SIM_NMEA = None


def sim_nmea_lines():
    """Yield 1 Hz fake NMEA (GSV, GGA, RMC) with valid checksums."""
    def wrap(body):
        c = 0
        for ch in body:
            c ^= ord(ch)
        return f"${body}*{c:02X}"
    k = 0
    while True:
        t = datetime.now(timezone.utc)
        hms = t.strftime("%H%M%S.00")
        yield wrap("GPGSV,1,1,04,05,45,120,42,12,30,200,38,18,60,300,45,25,15,050,33")
        ke, kn = min(k, 35), max(0, k - 35)          # L-shaped walk at 1.389 m/s: 35 s east, then north
        lon_min = 41.9100 - ke * 0.0010656
        lat_min = 23.0000 + kn * 0.00074977
        crs = "90.0" if k < 35 else "0.0"
        yield wrap(f"GPGGA,{hms},45{lat_min:07.4f},N,075{lon_min:07.4f},W,1,09,0.9,85.0,M,-34.0,M,,")
        yield wrap(f"GPRMC,{hms},A,45{lat_min:07.4f},N,075{lon_min:07.4f},W,2.7,{crs},"
                   f"{t.strftime('%d%m%y')},,,A")
        k += 1
        time.sleep(1.0)


# ======================================================================= logger
class Logger:
    def __init__(self, out_dir, sim):
        self.out_dir, self.sim = out_dir, sim
        self.stop = threading.Event()
        self.t0 = time.time()
        self.counts = {"imu": 0, "baro": 0, "gnss": 0, "gnss_fix": 0, "nmea_bad": 0, "lora": 0}
        self.lora_status = "off"
        self.files = {n: open(os.path.join(out_dir, f"{n}.csv"), "w", buffering=1)
                      for n in ("imu", "baro", "gnss", "lora", "events")}
        self.files["imu"].write("time_s,ax_mps2,ay_mps2,az_mps2,gx_radps,gy_radps,gz_radps,temp_c\n")
        self.files["baro"].write("time_s,pressure_pa,temp_c,alt_isa_m\n")
        self.files["gnss"].write("time_s,utc_hhmmss,lat_deg,lon_deg,height_m,fix_quality,"
                                 "num_sats,hdop,speed_mps,course_deg,mean_cn0_dbhz\n")
        self.files["lora"].write("time_s,rx_time_unix,from_id,portnum,rssi_dbm,snr_db,hop_start,"
                                 "hop_limit,sender_lat_deg,sender_lon_deg,sender_alt_m,pos_source\n")
        self.files["events"].write("time_s,event\n")
        self.lora_lock = threading.Lock()
        self.nmea = open(os.path.join(out_dir, "nmea.log"), "w", buffering=1)
        self.ev_lock = threading.Lock()
        self.fatal = None

    def now(self):
        return time.time() - self.t0

    def event(self, label):
        with self.ev_lock:
            self.files["events"].write(f"{self.now():.4f},{label}\n")

    # ---------------- I2C loop: IMU every 10 ms, BMP180 as a non-blocking state machine
    def i2c_loop(self):
        try:
            dev = SimI2C() if self.sim else RealI2C()
        except Exception as e:
            self.fatal = ("I2C sensors not answering: " + str(e) +
                          "\n  Run  i2cdetect -y 1  : you need 68 (IMU) and 77 (barometer)."
                          "\n  Check Pi pins 1, 3, 5, 6 -> breadboard rows 8, 5, 6, 7 (PDF section 5-6).")
            self.stop.set()
            return
        cal = dev.cal
        period = 1.0 / IMU_RATE_HZ
        next_t = time.monotonic()
        bmp_state, bmp_t, b5, last_temp_t, temp_c = "idle", 0.0, None, -1e9, None
        imu_f, baro_f = self.files["imu"], self.files["baro"]
        while not self.stop.is_set():
            try:
                acc, gyr, mpu_t = dev.read_mpu()
                ts = self.now()
                a, w = to_body(acc), to_body(gyr)
                imu_f.write(f"{ts:.4f},{a[0]:.5f},{a[1]:.5f},{a[2]:.5f},"
                            f"{w[0]:.6f},{w[1]:.6f},{w[2]:.6f},{mpu_t:.2f}\n")
                self.counts["imu"] += 1

                m = time.monotonic()
                if bmp_state == "idle":
                    if m - last_temp_t >= TEMP_EVERY_S or b5 is None:
                        dev.bmp_start_temp()
                        bmp_state, bmp_t = "temp", m
                    else:
                        dev.bmp_start_press()
                        bmp_state, bmp_t = "press", m
                elif bmp_state == "temp" and m - bmp_t >= 0.005:
                    b5 = bmp_b5(cal, dev.bmp_read_temp())
                    temp_c = ((b5 + 8) >> 4) / 10.0
                    last_temp_t, bmp_state = m, "idle"
                elif bmp_state == "press" and m - bmp_t >= 0.026:
                    p = bmp_pressure(cal, b5, dev.bmp_read_press(), BMP_OSS)
                    baro_f.write(f"{self.now():.4f},{p},{temp_c:.1f},{isa_alt(p):.3f}\n")
                    self.counts["baro"] += 1
                    bmp_state = "idle"
            except OSError as e:          # transient I2C glitch: log it and keep going
                self.event(f"i2c_error:{e.__class__.__name__}")
                time.sleep(0.01)
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # fell behind: resynchronise

    # ---------------- GPS thread
    def gps_loop(self):
        if self.sim:
            src = sim_nmea_lines()
            readline = lambda: next(src)
        else:
            try:
                import serial
                ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
            except Exception as e:
                print(f"\nWARNING: GPS not available ({e}). Recording IMU + barometer only.")
                self.event("gps_unavailable")
                return
            readline = lambda: ser.readline().decode("ascii", errors="replace").strip()
        cn0, gga = {}, None
        gf = self.files["gnss"]
        while not self.stop.is_set():
            line = readline()
            if not line or self.stop.is_set():
                continue
            ts = self.now()
            self.nmea.write(f"{ts:.4f},{line}\n")
            if not nmea_ok(line):
                self.counts["nmea_bad"] += 1
                continue
            f = line[1:].split("*")[0].split(",")
            kind = f[0][2:]
            if kind == "GSV":
                for i in range(4, len(f) - 3, 4):
                    if f[i]:
                        cn0[f[0][:2] + f[i]] = float(f[i + 3]) if f[i + 3] else None
            elif kind == "GGA":
                gga = (ts, f)                     # position part of this epoch
            elif kind == "RMC" and gga is not None and gga[1][1] == f[1]:
                # RMC follows GGA in the same epoch: write ONE row with position + velocity
                ts_g, g = gga
                q = int(g[6] or 0)
                vals = [v for v in cn0.values() if v is not None]
                mc = f"{sum(vals) / len(vals):.1f}" if vals else ""
                if q == 0 or not g[2]:
                    gf.write(f"{ts_g:.4f},{g[1]},,,,0,{int(g[7] or 0)},,,,{mc}\n")
                else:
                    lat, lon = nmea_deg(g[2], g[3], 2), nmea_deg(g[4], g[5], 3)
                    h = float(g[9] or 0) + float(g[11] or 0)     # MSL + geoid = ellipsoidal
                    spd = float(f[7] or 0) * 0.514444 if f[2] == "A" else 0.0
                    crs = float(f[8] or 0) if f[2] == "A" else 0.0
                    gf.write(f"{ts_g:.4f},{g[1]},{lat:.8f},{lon:.8f},{h:.3f},{q},{int(g[7] or 0)},"
                             f"{g[8]},{spd:.3f},{crs:.2f},{mc}\n")
                    self.counts["gnss_fix"] += 1
                self.counts["gnss"] += 1
                gga = None

    # ---------------- LoRa (Meshtastic) thread: RECEIVE-ONLY logging of every packet heard
    def lora_write(self, ts, pkt, node_pos=None):
        d = pkt.get("decoded", {}) or {}
        pos = d.get("position") if d.get("portnum") == "POSITION_APP" else None
        src = ""
        if pos and pos.get("latitude") is not None:
            lat, lon, alt, src = pos.get("latitude"), pos.get("longitude"), pos.get("altitude"), "packet"
        elif node_pos and node_pos.get("latitude") is not None:
            lat, lon, alt, src = node_pos.get("latitude"), node_pos.get("longitude"), node_pos.get("altitude"), "nodedb"
        else:
            lat = lon = alt = None
        f = lambda v, fmt="{}": "" if v is None else fmt.format(v)
        row = (f"{ts:.4f},{f(pkt.get('rxTime'))},{pkt.get('fromId') or pkt.get('from', '')},"
               f"{d.get('portnum', 'ENCRYPTED')},{f(pkt.get('rxRssi'))},{f(pkt.get('rxSnr'), '{:.2f}')},"
               f"{f(pkt.get('hopStart'))},{f(pkt.get('hopLimit'))},{f(lat, '{:.7f}')},{f(lon, '{:.7f}')},"
               f"{f(alt)},{src}\n")
        with self.lora_lock:
            self.files["lora"].write(row)
            self.counts["lora"] += 1

    def lora_loop(self, port=None):
        if self.sim:
            self.lora_status = "sim"
            nodes = {"!a1b2c3d4": (45.3870, -75.6960, 80), "!0badf00d": (45.3820, -75.7010, 75)}
            while not self.stop.wait(random.uniform(2.0, 6.0)):
                nid = random.choice(list(nodes))
                lat, lon, alt = nodes[nid]
                d_m = 300 + 200 * random.random()
                rssi = int(-40 - 28 * math.log10(d_m) + random.gauss(0, 4))      # simple path-loss model
                pkt = {"fromId": nid, "rxTime": int(time.time()), "rxRssi": rssi,
                       "rxSnr": round(random.uniform(-8, 10), 2), "hopStart": 3, "hopLimit": 3,
                       "decoded": {"portnum": random.choice(["POSITION_APP", "TELEMETRY_APP", "NODEINFO_APP"]),
                                   "position": {"latitude": lat, "longitude": lon, "altitude": alt}}}
                self.lora_write(self.now(), pkt, {"latitude": lat, "longitude": lon, "altitude": alt})
            return
        try:
            import meshtastic.serial_interface
            from pubsub import pub
        except ImportError:
            self.lora_status = "no library"
            self.event("lora_unavailable_no_library")
            return
        ports = [port] if port else [p for p in LORA_PORTS if os.path.exists(p)]
        if not ports:
            self.lora_status = "not plugged in"
            self.event("lora_unavailable_no_device")
            return
        iface_box = {}

        def on_receive(packet, interface):
            ts = self.now()
            try:
                me = interface.myInfo.my_node_num if interface.myInfo else None
                if me is not None and packet.get("from") == me:
                    return                      # our own node's packets: no RSSI, skip
                node = (interface.nodesByNum or {}).get(packet.get("from"), {})
                self.lora_write(ts, packet, node.get("position"))
            except Exception as e:
                self.event(f"lora_error:{e.__class__.__name__}")

        pub.subscribe(on_receive, "meshtastic.receive")
        try:
            iface_box["i"] = meshtastic.serial_interface.SerialInterface(devPath=ports[0])
        except Exception as e:
            print(f"\nWARNING: LoRa node on {ports[0]} not answering ({e}). Recording without LoRa.")
            self.lora_status = "error"
            self.event("lora_unavailable_error")
            return
        self.lora_status = f"ok ({ports[0]})"
        self.event("lora_connected")
        self.stop.wait()
        try:
            iface_box["i"].close()
        except Exception:
            pass

    def close(self):
        for f in self.files.values():
            f.close()
        self.nmea.close()


def write_readme(path, args, t0, counts, dur, notes, lora_status="off"):
    with open(path, "w") as f:
        f.write(f"""Team 14 DAQ run: {os.path.basename(os.path.dirname(path))}
{'SIMULATED SENSORS (--sim), NOT REAL DATA' if args.sim else 'Real sensor data'}
Start UTC     : {datetime.fromtimestamp(t0, timezone.utc).isoformat()}
Start epoch   : {t0:.6f}   (absolute time = start epoch + time_s)
Duration      : {dur:.1f} s
Rows          : imu {counts['imu']} ({counts['imu'] / max(dur, 1e-9):.1f} Hz), baro {counts['baro']} ({counts['baro'] / max(dur, 1e-9):.1f} Hz), gnss {counts['gnss']} ({counts['gnss_fix']} with fix), bad NMEA {counts['nmea_bad']}, lora packets {counts['lora']}
Sensors       : MPU6050 (+/-2 g, +/-250 deg/s, DLPF 44 Hz), BMP180 (OSS 3), Adafruit Ultimate GPS HAT (PA1616D, NMEA 9600 baud)
IMU frame     : body x forward, y right, z down (logger applies AXIS_MAP = {AXIS_MAP}); flat & still -> az ~ -9.81
Timestamps    : Pi system clock (chrony + GPS PPS when available); IMU spacing is not exactly 10 ms, use time_s
LoRa          : Meshtastic node (XIAO ESP32S3 + Wio-SX1262, 902-928 MHz, region US), receive-side log of
                RSSI/SNR per packet heard; sender position from the packet or the node database (pos_source)
GNSS          : height = MSL + geoid separation (ellipsoidal); empty fields = no fix
Lever arm     : IMU -> GPS antenna (x fwd, y right, z down, cm): FILL IN
LoRa status   : {lora_status}
Route / notes : {notes or 'FILL IN'}
""")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="", help="label added to the run folder name")
    ap.add_argument("--out", default=os.path.expanduser("~/runs"))
    ap.add_argument("--sim", action="store_true", help="fake sensors, no hardware needed")
    ap.add_argument("--no-lora", action="store_true", help="do not use the Meshtastic LoRa node")
    ap.add_argument("--lora-port", default=None, help="LoRa node serial port (default: auto /dev/ttyACM0)")
    ap.add_argument("--lte", default="", help="also record LTE bursts on these carriers, e.g. 885.1,875.2")
    ap.add_argument("--lte-gain", type=float, default=30.0)
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = until q)")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = os.path.join(args.out, f"run_{stamp}" + (f"_{args.name}" if args.name else ""))
    os.makedirs(out_dir, exist_ok=True)
    lg = Logger(out_dir, args.sim)
    lg.event("logger_start")
    threads = [threading.Thread(target=lg.i2c_loop, daemon=True),
               threading.Thread(target=lg.gps_loop, daemon=True)]
    if not args.no_lora:
        threads.append(threading.Thread(target=lg.lora_loop, args=(args.lora_port,), daemon=True))
    for th in threads:
        th.start()
    lte_proc = None
    if args.lte:
        cmd = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lte_capture.py"),
               "--out", out_dir, "--t0", repr(lg.t0), "--freqs", args.lte, "--gain", str(args.lte_gain)]
        if args.sim:
            cmd.append("--sim")
        lte_proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL)   # separate process: never slows the IMU
        lg.event("lte_start")
    print(f"Recording to {out_dir}")
    print("Type an event label + Enter (e.g. static_end, tunnel_in, stairs_up). q + Enter = stop.")

    inputs = queue.Queue()
    if args.duration <= 0:
        threading.Thread(target=lambda: [inputs.put(l.strip()) for l in sys.stdin], daemon=True).start()
    t_last = 0
    try:
        while True:
            if lg.fatal:
                print("\nSTOPPED: " + lg.fatal)
                break
            if args.duration > 0 and lg.now() >= args.duration:
                break
            try:
                cmd = inputs.get(timeout=0.5)
                if cmd.lower() in ("q", "quit", "stop"):
                    break
                if cmd:
                    lg.event(cmd.replace(",", "_"))
                    print(f"  event '{cmd}' at t = {lg.now():.1f} s")
            except queue.Empty:
                pass
            if lg.now() - t_last >= 10:
                t_last = lg.now()
                c = lg.counts
                print(f"  t={t_last:6.0f}s  imu {c['imu']}  baro {c['baro']}  gnss {c['gnss']} "
                      f"(fix {c['gnss_fix']})  lora {c['lora']} [{lg.lora_status}]")
    except KeyboardInterrupt:
        pass
    lg.event("logger_stop")
    dur = lg.now()
    if lte_proc is not None:
        open(os.path.join(out_dir, "lte_stop"), "w").close()
        try:
            lte_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            lte_proc.terminate()
    lg.stop.set()
    for th in threads:
        th.join(timeout=2.5)          # let GPS/I2C/LoRa threads finish their last write
    lg.close()
    write_readme(os.path.join(out_dir, "README.txt"), args, lg.t0, lg.counts, dur, args.name, lg.lora_status)
    try:                                   # one combined CSV at the IMU rate for the fusion team
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        bpath = os.path.join(out_dir, "lte_bursts.csv")
        if args.lte and os.path.exists(bpath):
            nb = sum(1 for _ in open(bpath)) - 1
            if os.environ.get("DAQ_PROCESS_ON_PI") == "1" and nb <= 400:
                print(f"  processing {nb} LTE bursts (about a minute)...")
                import lte_process
                lte_process.process(out_dir)
            else:
                print(f"  {nb} LTE bursts saved. LTE is processed on the LAPTOP (too slow on the Pi):\n"
                      f"  Process later (laptop is much faster):  python3 lte_process.py {out_dir}\n"
                      f"  then:                                   python3 merge_run.py {out_dir}")
        import merge_run
        merge_run.merge(out_dir)
    except Exception as e:
        print(f"  combined.csv not made ({e}); run:  python3 merge_run.py {out_dir}")
    c = lg.counts
    print(f"\nSaved {out_dir}\n  {dur:.1f} s | imu {c['imu']} ({c['imu'] / dur:.1f} Hz) | "
          f"baro {c['baro']} ({c['baro'] / dur:.1f} Hz) | gnss {c['gnss']} ({c['gnss_fix']} fixes) | lora {c['lora']} packets [{lg.lora_status}]")
    print("  Edit README.txt to add the route notes and the lever arm.")


if __name__ == "__main__":
    main()
