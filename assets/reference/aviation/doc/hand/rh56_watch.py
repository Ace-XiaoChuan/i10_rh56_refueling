import serial
import struct
import time
import argparse
from datetime import datetime

BAUD = 115200

ANGLE_ACT = 1546
FORCE_ACT = 1582
CURRENT   = 1594
ERROR     = 1606
STATUS    = 1612
TEMP      = 1618

FINGER_NAMES = ["little", "ring", "middle", "index", "thumb_bend", "thumb_rotate"]

def checksum(data):
    return sum(data) & 0xFF

def build_read(hand_id, addr, length):
    frame = [
        0xEB, 0x90,
        hand_id,
        0x04,
        0x11,
        addr & 0xFF,
        (addr >> 8) & 0xFF,
        length
    ]
    frame.append(checksum(frame[2:]))
    return bytes(frame)

def read_register(ser, hand_id, addr, length):
    ser.reset_input_buffer()
    ser.write(build_read(hand_id, addr, length))
    ser.flush()
    time.sleep(0.025)

    resp = ser.read(8 + length)

    if len(resp) != 8 + length:
        raise RuntimeError(f"short response: {resp.hex(' ')}")

    if resp[0] != 0x90 or resp[1] != 0xEB:
        raise RuntimeError(f"bad header: {resp.hex(' ')}")

    if resp[2] != hand_id:
        raise RuntimeError(f"wrong id: {resp.hex(' ')}")

    calc = checksum(resp[2:-1])
    if calc != resp[-1]:
        raise RuntimeError(f"bad checksum: {resp.hex(' ')}")

    return resp[7:7 + length]

def read_six_shorts(ser, hand_id, addr):
    data = read_register(ser, hand_id, addr, 12)
    return list(struct.unpack("<hhhhhh", data))

def read_six_bytes(ser, hand_id, addr):
    data = read_register(ser, hand_id, addr, 6)
    return list(data)

def fmt_list(values):
    return "[" + " ".join(f"{v:5d}" for v in values) + "]"

def gf_to_n(gf):
    return gf * 0.00980665

def fmt_force_n(values):
    return "[" + " ".join(f"{gf_to_n(v):6.2f}" for v in values) + "]"

def main():
    parser = argparse.ArgumentParser(description="Realtime RH56 state watcher")
    parser.add_argument("--port", default="/dev/ttyUSB_robot_485")
    parser.add_argument("--id", type=int, action="append", help="hand id, e.g. --id 1 --id 2")
    parser.add_argument("--both", action="store_true", help="watch id 1 and id 2")
    parser.add_argument("--rate", type=float, default=10.0, help="print rate Hz")
    parser.add_argument("--clear", action="store_true", help="dashboard mode, not scrolling")
    args = parser.parse_args()

    if args.both:
        hand_ids = [1, 2]
    elif args.id:
        hand_ids = args.id
    else:
        hand_ids = [2]

    dt = 1.0 / max(1.0, args.rate)

    print("RH56 realtime watcher")
    print(f"port={args.port}, ids={hand_ids}, rate={args.rate}Hz")
    print("force displayed in Newton (N), converted from gf")
    print("Press Ctrl+C to stop.\n")

    baselines = {}

    with serial.Serial(args.port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.2) as ser:
        for hid in hand_ids:
            try:
                baselines[hid] = read_six_shorts(ser, hid, FORCE_ACT)
                print(f"baseline hand {hid}: {baselines[hid]}")
            except Exception as e:
                print(f"baseline hand {hid} failed: {e}")
                baselines[hid] = [0, 0, 0, 0, 0, 0]

        print("\nColumns:")
        print("time | id | force(N) | dF(N) | angle | temp | error")
        print("-" * 140)

        while True:
            if args.clear:
                print("\033[2J\033[H", end="")

            now = datetime.now().strftime("%H:%M:%S.%f")[:-3]

            for hid in hand_ids:
                try:
                    angles = read_six_shorts(ser, hid, ANGLE_ACT)
                    forces = read_six_shorts(ser, hid, FORCE_ACT)
                    temps  = read_six_bytes(ser, hid, TEMP)
                    errors = read_six_bytes(ser, hid, ERROR)

                    df = [f - b for f, b in zip(forces, baselines[hid])]

                    print(f"{now} | id={hid} | force {fmt_force_n(forces)} | dF {fmt_force_n(df)} | angle {fmt_list(angles)} | temp {temps} | err {errors}")

                except Exception as e:
                    print(f"{now} | id={hid} | READ FAILED: {e}")

            time.sleep(dt)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")