#!/bin/bash
# LimeSDR Mini 2.0 receive setup for the Team 14 Pi.   Run once:   bash setup_sdr.sh
set -e
echo "== 1/3 Packages (SoapySDR + LimeSuite driver + numpy/scipy)"
sudo apt update
sudo apt install -y python3-numpy python3-scipy soapysdr-tools python3-soapysdr \
                    limesuite limesuite-udev soapysdr-module-lms7 || true

echo "== 2/3 Is the LimeSDR seen?  (plug it into a BLUE USB 3.0 port, antenna on RX)"
SoapySDRUtil --find || true
echo
echo "You should see a block with  driver = lime  and  LimeSDR Mini  in it."
echo "If NOT:"
echo "  - LimeUtil --find          (does the low-level driver see it?)"
echo "  - LimeUtil --update        (updates the board firmware/gateware, takes ~1 min)"
echo "  - if the apt driver is too old for the Mini 2.0, build LimeSuiteNG:"
echo "      git clone https://github.com/myriadrf/LimeSuiteNG && cd LimeSuiteNG"
echo "      sudo ./install_dependencies.sh && cmake -B build && cd build && make -j4 && sudo make install"
echo
echo "== 3/3 Quick test without hardware"
python3 "$(dirname "$0")/lte_scan.py" --sim --label setup_test | tail -12
echo
echo "Done. Real scan:  python3 lte_scan.py --label where_you_stand"
