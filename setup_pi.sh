#!/bin/bash
# One-time setup for the Team 14 DAQ Pi (Raspberry Pi 4, Raspberry Pi OS Bookworm).
# Run once after first SSH login:   bash setup_pi.sh     (then it reboots)
set -e
CFG=/boot/firmware/config.txt
[ -f "$CFG" ] || CFG=/boot/config.txt

echo "== 1/5 Packages"
sudo apt update
sudo apt install -y python3-smbus2 python3-serial python3-matplotlib i2c-tools pps-tools chrony
# Optional LoRa node (Meshtastic): Python library + serial-port permission
pip install meshtastic --break-system-packages || echo "meshtastic install failed (LoRa is optional)"
sudo usermod -aG dialout "$USER"

echo "== 2/5 Enable I2C, enable serial hardware, disable serial login console"
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1

echo "== 3/5 Give the good UART to the GPS (disable Bluetooth) + PPS on GPIO4"
grep -q "^dtoverlay=disable-bt" "$CFG" || echo "dtoverlay=disable-bt" | sudo tee -a "$CFG"
grep -q "^dtoverlay=pps-gpio" "$CFG" || echo "dtoverlay=pps-gpio,gpiopin=4" | sudo tee -a "$CFG"
grep -q "^enable_uart=1" "$CFG" || echo "enable_uart=1" | sudo tee -a "$CFG"
sudo systemctl disable hciuart 2>/dev/null || true

echo "== 4/5 chrony: use GPS PPS for sub-microsecond timing (internet NTP gives the seconds)"
echo "refclock PPS /dev/pps0 refid PPS precision 1e-7 prefer" | sudo tee /etc/chrony/conf.d/gps-pps.conf
sudo systemctl enable chrony

echo "== 5/5 Folder for recordings"
mkdir -p ~/runs

echo "Done. Rebooting in 5 s. After reboot run:  python3 sensor_check.py all"
sleep 5
sudo reboot
