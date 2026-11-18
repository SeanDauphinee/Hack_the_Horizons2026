#!/usr/bin/env python3
"""Quick sensor sanity check for the DAQ rig (Raspberry Pi 4).

Usage (on the Pi):
    python3 sensor_check.py imu     # MPU6050: identity, gravity magnitude, gyro at rest
    python3 sensor_check.py baro    # BMP180: identity, pressure, altitude, temperature
    python3 sensor_check.py gps     # GPS HAT: NMEA stream, fix, satellites, position
    python3 sensor_check.py all     # one identity check per sensor, then exit

Needs:  sudo apt install python3-smbus2 python3-serial   (and I2C + serial enabled)
Stop a live test with Ctrl+C.
"""
import math
import sys
import time

I2C_BUS = 1
MPU_ADDR = 0x68
BMP_ADDR = 0x77
GPS_PORT = "/dev/serial0"
GPS_BAUD = 9600
G = 9.80665


def ok(cond):
    return "PASS" if cond else "CHECK"


# ------------------------------------------------------------------ MPU6050
def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def mpu_init(bus):
    who = bus.read_byte_data(MPU_ADDR, 0x75)       # WHO_AM_I, expect 0x68
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x00)      # wake up (clear sleep bit)
    bus.write_byte_data(MPU_ADDR, 0x1A, 0x03)      # digital low-pass ~44 Hz
    bus.write_byte_data(MPU_ADDR, 0x1B, 0x00)      # gyro  +/-250 deg/s -> 131 LSB/(deg/s)
    bus.write_byte_data(MPU_ADDR, 0x1C, 0x00)      # accel +/-2 g        -> 16384 LSB/g
    time.sleep(0.1)
    return who


def mpu_read(bus):
    d = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)
    ax, ay, az = (s16(d[i], d[i + 1]) / 16384.0 * G for i in (0, 2, 4))
    temp = s16(d[6], d[7]) / 340.0 + 36.53
    gx, gy, gz = (s16(d[i], d[i + 1]) / 131.0 for i in (8, 10, 12))   # deg/s
    return ax, ay, az, gx, gy, gz, temp


def check_imu(live=True):
    from smbus2 import SMBus
    with SMBus(I2C_BUS) as bus:
        who = mpu_init(bus)
        print(f"MPU6050 WHO_AM_I = 0x{who:02X}  [{ok(who == 0x68)}]  (expect 0x68)")
        if not live:
            return
        print("Keep the board STILL and flat for the first readings, then tilt / rotate it.")
        print("   ax      ay      az    |a| m/s2   gx     gy     gz deg/s   temp C")
        while True:
            ax, ay, az, gx, gy, gz, t = mpu_read(bus)
            mag = math.sqrt(ax * ax + ay * ay + az * az)
            print(f"{ax:7.2f} {ay:7.2f} {az:7.2f}  {mag:6.2f} [{ok(abs(mag - G) < 0.5)}]"
                  f"  {gx:6.2f} {gy:6.2f} {gz:6.2f}   {t:5.1f}")
            time.sleep(0.2)


# ------------------------------------------------------------------ BMP180
def bmp_calibration(raw22):
    """22 calibration bytes (0xAA..0xBF) -> dict of the 11 coefficients."""
    names = ["AC1", "AC2", "AC3", "AC4", "AC5", "AC6", "B1", "B2", "MB", "MC", "MD"]
    unsigned = {"AC4", "AC5", "AC6"}
    cal = {}
    for i, n in enumerate(names):
        v = (raw22[2 * i] << 8) | raw22[2 * i + 1]
        if n not in unsigned and v > 32767:
            v -= 65536
        cal[n] = v
    return cal


def _div(a, b):
    """C-style integer division (truncates toward zero), as the datasheet assumes."""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def bmp_compensate(cal, ut, up, oss):
    """Bosch BMP180 datasheet algorithm. Returns (temperature degC, pressure Pa)."""
    x1 = ((ut - cal["AC6"]) * cal["AC5"]) >> 15
    x2 = _div(cal["MC"] << 11, x1 + cal["MD"])
    b5 = x1 + x2
    t = (b5 + 8) >> 4                                   # 0.1 degC
    b6 = b5 - 4000
    x1 = (cal["B2"] * ((b6 * b6) >> 12)) >> 11
    x2 = (cal["AC2"] * b6) >> 11
    x3 = x1 + x2
    b3 = _div(((cal["AC1"] * 4 + x3) << oss) + 2, 4)
    x1 = (cal["AC3"] * b6) >> 13
    x2 = (cal["B1"] * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (cal["AC4"] * ((x3 + 32768) & 0xFFFFFFFF)) >> 15
    b7 = ((up - b3) & 0xFFFFFFFF) * (50000 >> oss)
    p = (b7 * 2) // b4 if b7 < 0x80000000 else (b7 // b4) * 2
    x1 = (p >> 8) * (p >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * p) >> 16
    p = p + ((x1 + x2 + 3791) >> 4)
    return t / 10.0, p


def bmp_read(bus, cal, oss=3):
    bus.write_byte_data(BMP_ADDR, 0xF4, 0x2E)           # start temperature
    time.sleep(0.005)
    d = bus.read_i2c_block_data(BMP_ADDR, 0xF6, 2)
    ut = (d[0] << 8) | d[1]
    bus.write_byte_data(BMP_ADDR, 0xF4, 0x34 + (oss << 6))   # start pressure
    time.sleep(0.026)
    d = bus.read_i2c_block_data(BMP_ADDR, 0xF6, 3)
    up = ((d[0] << 16) | (d[1] << 8) | d[2]) >> (8 - oss)
    return bmp_compensate(cal, ut, up, oss)


def check_baro(live=True):
    from smbus2 import SMBus
    with SMBus(I2C_BUS) as bus:
        cid = bus.read_byte_data(BMP_ADDR, 0xD0)
        print(f"BMP180 chip ID = 0x{cid:02X}  [{ok(cid == 0x55)}]  (expect 0x55)")
        if not live:
            return
        cal = bmp_calibration(bus.read_i2c_block_data(BMP_ADDR, 0xAA, 22))
        print("Expect ~97-103 kPa in Ottawa, temp near room temperature.")
        print("Lift the board ~1 m: pressure should DROP by ~12 Pa (altitude up ~1 m).")
        print("  temp C   pressure Pa   altitude m (ISA)")
        while True:
            t, p = bmp_read(bus, cal)
            alt = 44330.8 * (1.0 - (p / 101325.0) ** 0.190263)
            print(f"  {t:6.1f}   {p:10d}  [{ok(95000 < p < 105000)}]   {alt:7.2f}")
            time.sleep(0.2)


# ------------------------------------------------------------------ GPS
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


def check_gps(live=True):
    import serial
    with serial.Serial(GPS_PORT, GPS_BAUD, timeout=2) as ser:
        good = bad = 0
        t_end = time.time() + 3
        while time.time() < t_end:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if line:
                good, bad = (good + 1, bad) if nmea_ok(line) else (good, bad + 1)
        print(f"GPS serial: {good} valid NMEA lines, {bad} bad in 3 s  [{ok(good > 5)}]")
        if not live:
            return
        print("Fix LED: blinks ~1 s = searching, blinks every ~15 s = FIX. Go outdoors.")
        while True:
            line = ser.readline().decode("ascii", errors="replace").strip()
            if nmea_ok(line) and line[3:6] == "GGA":
                f = line.split(",")
                q, n = int(f[6] or 0), int(f[7] or 0)
                if q == 0:
                    print(f"No fix yet, satellites in use: {n}")
                else:
                    lat = float(f[2][:2]) + float(f[2][2:]) / 60 * 1
                    lat = -lat if f[3] == "S" else lat
                    lon = float(f[4][:3]) + float(f[4][3:]) / 60
                    lon = -lon if f[5] == "W" else lon
                    print(f"FIX q={q} sats={n} HDOP={f[8]}  lat {lat:.6f} lon {lon:.6f} "
                          f"alt {f[9]} m  [{ok(n >= 5)}]  -> compare with your phone's map")


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    try:
        if what == "imu":
            check_imu()
        elif what == "baro":
            check_baro()
        elif what == "gps":
            check_gps()
        else:
            for name, fn in (("IMU", check_imu), ("BARO", check_baro), ("GPS", check_gps)):
                try:
                    fn(live=False)
                except Exception as e:
                    print(f"{name}: FAILED -> {e}")
    except KeyboardInterrupt:
        print("\nstopped")
