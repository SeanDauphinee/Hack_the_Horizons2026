#!/usr/bin/env python3
"""Check the Meshtastic LoRa node (Seeed XIAO ESP32S3 + Wio-SX1262) plugged into the Pi by USB.

    python3 lora_check.py            # show node info + region, then listen 60 s and print every packet
    python3 lora_check.py 300        # listen 300 s
    python3 lora_check.py --sim      # no hardware

Receive-only: this script never transmits a message.
"""
import os
import sys
import time

PORTS = ("/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    secs = float(args[0]) if args else 60
    if "--sim" in sys.argv:
        print("SIM: pretend node !a1b2c3d4 heard at -95 dBm, SNR 4.5 dB. [ OK ]")
        return
    try:
        import meshtastic.serial_interface
        from pubsub import pub
    except ImportError:
        print("Meshtastic library missing. Install it with:\n"
              "  pip install meshtastic --break-system-packages")
        sys.exit(1)
    ports = [p for p in PORTS if os.path.exists(p)]
    if not ports:
        print("No LoRa node found (no /dev/ttyACM0).\n"
              "  - Plug the XIAO into a Pi USB port with the USB-C cable (use a DATA cable).\n"
              "  - Then run:  ls /dev/ttyACM*")
        sys.exit(1)

    heard = []

    def on_rx(packet, interface):
        me = interface.myInfo.my_node_num if interface.myInfo else None
        if packet.get("from") == me:
            return
        d = packet.get("decoded", {}) or {}
        heard.append(packet)
        pos = d.get("position") or {}
        extra = f"  pos {pos.get('latitude'):.5f},{pos.get('longitude'):.5f}" if pos.get("latitude") else ""
        print(f"  {time.strftime('%H:%M:%S')}  from {packet.get('fromId')}  {d.get('portnum', 'ENCRYPTED'):14s}"
              f"  RSSI {packet.get('rxRssi')} dBm  SNR {packet.get('rxSnr')} dB"
              f"  hops {packet.get('hopStart')}->{packet.get('hopLimit')}{extra}")

    pub.subscribe(on_rx, "meshtastic.receive")
    print(f"Connecting to {ports[0]} ...")
    iface = meshtastic.serial_interface.SerialInterface(devPath=ports[0])
    try:
        u = iface.getMyUser() or {}
        print(f"  My node      : {u.get('longName')} ({u.get('id')}), hardware {u.get('hwModel')}")
        try:
            lora = iface.localNode.localConfig.lora
            try:
                from meshtastic.protobuf import config_pb2
            except ImportError:
                from meshtastic import config_pb2
            region = config_pb2.Config.LoRaConfig.RegionCode.Name(lora.region)
            role = config_pb2.Config.DeviceConfig.Role.Name(iface.localNode.localConfig.device.role)
            print(f"  Region       : {region}   (must be US for Canada = 902-928 MHz)")
            print(f"  Role         : {role}   (CLIENT_MUTE = does not relay other people's packets)")
            print(f"  Modem preset : {config_pb2.Config.LoRaConfig.ModemPreset.Name(lora.modem_preset)}")
            if region != "US":
                print("  [ CHECK ] Region is not US. Fix it once with:\n"
                      "            meshtastic --set lora.region US")
        except Exception as e:
            print(f"  (could not read radio config: {e})")
        others = [n for n in (iface.nodes or {}).values() if n.get("num") != iface.myInfo.my_node_num]
        print(f"  Nodes already known: {len(others)}")
        print(f"\nListening {secs:.0f} s for packets (Ctrl+C to stop early) ...")
        try:
            time.sleep(secs)
        except KeyboardInterrupt:
            pass
        print(f"\nHeard {len(heard)} packets from {len({p.get('fromId') for p in heard})} nodes.")
        print("[ OK ] node works" if heard else
              "[ INFO ] node works but heard nothing. Normal if nobody nearby runs Meshtastic:\n"
              "         put a 2nd node (Sean / a phone-paired node) a few hundred m away as a beacon.")
    finally:
        iface.close()


if __name__ == "__main__":
    main()
