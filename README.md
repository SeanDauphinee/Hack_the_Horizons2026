# Hack the Horizons 2026 — Team 14

**1st place, Honeywell × Carleton University "Hack the Horizons" hackathon (November 2026)**
Challenge 2: positioning and navigation when GPS is unavailable.

Team 14: Tony Kairouz, Gabe Melendez, Tristan Laliberté, Sean Dauphinee, Tyler Laliberté

## Motivation

GNSS jamming is now a real problem for commercial aviation. Between January and April 2025, about 123,000 flights
over the Baltic region experienced GNSS interference. GNSS signals reach aircraft very weakly, so they are easy to
jam or spoof.

**Goal:** keep an aircraft navigating, and report how much the position can be trusted, when GNSS is lost.

We started with small fixed-wing UAVs, then rescoped to **commercial aircraft** after reviews with an aerospace
professor and a Honeywell mentor: smoother dynamics, clear Transport Canada requirements, and a larger market.

| Phase | Transport Canada accuracy requirement (95 % of flight time) |
|---|---|
| Cruise (RNAV 2) | ≤ 2 NM (3.7 km) |
| Near airport | ≤ 1 NM (1.85 km) |
| Approach | ≤ 0.3 NM (556 m) |
| Integrity | the system must alert the crew when it cannot guarantee the required accuracy |

Source: Transport Canada Aeronautical Information Manual (TC AIM), COM 5.3.1, 6.3.3, 6.3.6, 6.3.7, 6.6.

## Approach

An **INS (IMU) + barometer** core, aided by **signals of opportunity**: LTE 4G cell towers (wide coverage, fixed
known locations, low frequency) and, in the architecture, LEO satellites (Iridium, Starlink, Kuiper). An extended
Kalman filter fuses them and outputs position plus a confidence level.

Two parts:
1. **Simulation (MATLAB/Simulink):** an Ottawa → Toronto (YOW → YYZ) flight with a 10-minute GNSS outage in cruise
   (25,000 ft) and a 5-minute outage on final approach, with realistic IMU/barometer noise and bias and simulated
   LTE tower signals (correlator → pseudo-ranges → EKF). *(Simulation code not included in this repository.)*
2. **Physical demo (this repository):** a real sensor rig that records the same kinds of measurements, so the
   filter can be tested on real data.

This repository contains the **data-acquisition (DAQ) system**: a Raspberry Pi rig that records an
IMU, a barometer, a GPS receiver and real **LTE cell-tower signals** on one clock, and the tools that turn
each recording into a single CSV in the filter's input format.

---

## What it does

```
 MPU6050 IMU ──I2C──┐
 BMP180 baro ──I2C──┤                               ┌─ lte_process.py (LTE timing, cell IDs)
 GPS HAT ─UART+PPS──┼─> Raspberry Pi 4 ─> run folder ┼─ merge_run.py   → combined.csv ─> MATLAB filter
 LimeSDR ─USB 2.0───┘   (daq.py menu)    (raw CSVs)  └─ check_run.py   → quick-look plots
```

| Sensor | Rate | Notes |
|---|---|---|
| MPU6050 IMU | 100 Hz | ±2 g, ±250 °/s, body frame x forward / y right / z down |
| BMP180 barometer | ~25 Hz | pressure-altitude |
| Adafruit Ultimate GPS HAT | 1 Hz + PPS | PPS disciplines the Pi clock (chrony) |
| LimeSDR Mini 2.0 (receive only) | 40 ms burst / s | LTE Band 5 (875.2 MHz) as a *signal of opportunity* |

**LTE signal of opportunity.** The LimeSDR captures short bursts of the LTE downlink. `lte_dsp.py` detects the
primary and secondary synchronization signals (PSS/SSS) to read each cell's **Physical Cell ID** and measure its
**arrival time**, which becomes a pseudo-range. The radio only listens and never transmits.

**Output.** Every recording produces `combined.csv`: one row per IMU sample, with columns in the same order as the
team's Simulink measurement bus (`Meas_Bus`): IMU, barometer, LTE ranges, GNSS position/velocity (NED),
valid flags and timestamps. `load_meas_bus.m` loads it straight into MATLAB.

---

## Results

| Run | Distance | GPS fix | Notes |
|---|---|---|---|
| Walking loop around campus | 759 m | 100 % (median 10 satellites) | one LTE cell tracked 88 % of the time; its range follows the distance to the nearest Rogers tower |
| Car drive (campus, Bronson, canal) | 4.5 km, up to 64 km/h | 100 % (median 14 satellites) | 7 LTE cells decoded while driving |

Lessons learned:
- The LimeSDR on a **USB 3.0** port jammed the GPS (0 satellites). Moving it to **USB 2.0** fixed it.
- The LimeSDR sometimes tunes 100–200 kHz off frequency; `lte_process.py` detects and corrects it.
- The LimeSDR's free-running clock dominates LTE range error (hundreds of metres). Locking it to the GPS PPS, or
  sampling the sensors and the SDR on one shared computer clock, is the next hardware step.
- Only one antenna was available, so one LTE band (Band 5, 875.2 MHz) was tracked at a time.
- Data was copied over Wi-Fi; future tests should use Ethernet so Wi-Fi bands can be studied without cross-talk.

## Path to a product (TRL 9)
- Larger antennas covering a wider band of signals
- ARINC avionics communication interfaces
- EMC certification
- Flight testing, and a safe way to simulate GNSS denial without affecting other air traffic

---

## Hardware

Raspberry Pi 4, MPU6050 (GY-521), BMP180, Adafruit Ultimate GPS HAT with external antenna, LimeSDR Mini 2.0 with an
868–915 MHz antenna, 5 V / 3 A power. Wiring: [`docs/Team14_Wiring_and_Integration.pdf`](docs/Team14_Wiring_and_Integration.pdf)

| Pi pin | Connection |
|---|---|
| 1 (3.3 V), 9 (GND) | IMU + barometer power |
| 3 (SDA), 5 (SCL) | I2C bus 1: MPU6050 0x68, BMP180 0x77 |
| 2 (5 V), 6 (GND) | GPS HAT power |
| 8 (TXD) → GPS RX, 10 (RXD) ← GPS TX | UART `/dev/serial0`, 9600 baud |
| 7 (GPIO4) | GPS PPS |
| USB 2.0 (black) | LimeSDR, **not** USB 3.0 |

---

## Usage

### 1. Set up the Pi (once)
```bash
bash setup_pi.sh      # I2C, UART, PPS, chrony; reboots
bash setup_sdr.sh     # LimeSDR driver (SoapySDR + LimeSuite)
```

### 2. Record (on the Pi)
```bash
python3 daq.py        # menu: 1-4 sensor checks, 5 record a run, 9 LTE tower scan
```
While recording, stand still ~2 min, type `static_end`, move, type labels for key moments (e.g. `building_in`),
stand still 30 s, then `q`. Each run is saved to `~/runs/run_<date>_<time>_<name>/`.

### 3. Process (on a laptop; LTE processing is too heavy for the Pi)
```bash
RUN=run_20261111_151431_run
scp -r team14@<pi-ip>:~/runs/$RUN ~/Downloads/
python3 lte_process.py ~/Downloads/$RUN [--link 308=C5025]
python3 merge_run.py   ~/Downloads/$RUN
python3 check_run.py   ~/Downloads/$RUN
```
`--link PCI=TOWER` ties an LTE cell to a tower in `lte_towers.csv`. Without it, the script suggests a tower by
comparing the cell's range changes with the GPS track.

### 4. Load in MATLAB
```matlab
[meas, T] = load_meas_bus('combined.csv');
```

---

## Files

| File | Purpose |
|---|---|
| `daq.py` | terminal menu for checks and recording |
| `daq_logger.py` | recorder: IMU/baro thread, GPS thread, LTE capture process, events |
| `sensor_check.py` | live sensor checks |
| `lte_capture.py` | LimeSDR burst recorder (receive only) |
| `lte_scan.py` | LTE band scan: finds carriers and cell IDs |
| `lte_dsp.py` | PSS/SSS detection, timing, frequency offset |
| `lte_process.py` | per-run LTE processing → `lte.csv` |
| `lte_towers.csv` | Rogers Band 5 sites near Carleton (public tower data) |
| `merge_run.py` | builds `combined.csv` in the `Meas_Bus` layout |
| `check_run.py` | data-quality report + `quicklook.png` |
| `load_meas_bus.m` | MATLAB loader for `combined.csv` |
| `lora_check.py` | optional Meshtastic LoRa node check (uses the GPL-3.0 `meshtastic` library) |
| `setup_pi.sh`, `setup_sdr.sh` | one-time setup scripts |

## Team roles
- **Data acquisition and integration (physical demo):** Tony Kairouz, Sean Dauphinee
- **Sensor fusion and simulation (MATLAB/Simulink):** Tristan Laliberté
- **Research, presentation and documentation:** Gabe Melendez, Tyler Laliberté
  (requirements, architecture, presentation and the many details that tied the project together)

## License
MIT, see [`LICENSE`](LICENSE). Shared with the agreement of the hackathon organizers.
