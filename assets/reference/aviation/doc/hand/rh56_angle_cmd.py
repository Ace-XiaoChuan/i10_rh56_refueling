import serial
import struct
import time
import argparse

BAUD = 115200

ANGLE_SET = 1486
ANGLE_ACT = 1546
FORCE_ACT = 1582
ERROR = 1606
TEMP = 1618

FINGER_NAMES = [
    "little",
    "ring",
    "middle",
    "index",
    "thumb_bend",
    "thumb_rotate"
]

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

def build_write(hand_id, addr, data):
    frame = [
        0xEB, 0x90,
        hand_id,
        len(data) + 3,
        0x12,
        addr & 0xFF,
        (addr >> 8) & 0xFF
    ] + list(data)
    frame.append(checksum(frame[2:]))
    return bytes(frame)

def read_register(ser, hand_id, addr, length):
    ser.reset_input_buffer()
    ser.write(build_read(hand_id, addr, length))
    ser.flush()
    time.sleep(0.05)

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

def write_register(ser, hand_id, addr, data):
    ser.reset_input_buffer()
    ser.write(build_write(hand_id, addr, data))
    ser.flush()
    time.sleep(0.08)

    resp = ser.read(9)

    if len(resp) != 9:
        raise RuntimeError(f"write short response: {resp.hex(' ')}")

    if resp[0] != 0x90 or resp[1] != 0xEB:
        raise RuntimeError(f"write bad header: {resp.hex(' ')}")

    if resp[2] != hand_id:
        raise RuntimeError(f"write wrong id: {resp.hex(' ')}")

    calc = checksum(resp[2:-1])
    if calc != resp[-1]:
        raise RuntimeError(f"write bad checksum: {resp.hex(' ')}")

    return resp

def read_six_shorts(ser, hand_id, addr):
    data = read_register(ser, hand_id, addr, 12)
    return list(struct.unpack("<hhhhhh", data))

def read_six_bytes(ser, hand_id, addr):
    data = read_register(ser, hand_id, addr, 6)
    return list(data)

def clamp(v):
    v = int(v)
    if v == -1:
        return -1
    return max(0, min(1000, v))

def pack_six(values):
    return b"".join(struct.pack("<h", clamp(v)) for v in values)

def print_state(ser, hand_id, title):
    angles = read_six_shorts(ser, hand_id, ANGLE_ACT)
    forces = read_six_shorts(ser, hand_id, FORCE_ACT)
    errors = read_six_bytes(ser, hand_id, ERROR)
    temps = read_six_bytes(ser, hand_id, TEMP)

    print(f"\n[{title}] hand_id={hand_id}")
    print("finger           angle    force    temp")
    print("----------------------------------------")
    for name, a, f, t in zip(FINGER_NAMES, angles, forces, temps):
        print(f"{name:13s} {a:6d} {f:8d} {t:7d}")
    print("errors:", errors)

def main():
    parser = argparse.ArgumentParser(description="RH56 angle command with state display")
    parser.add_argument(
        "angles",
        nargs=6,
        type=int,
        help="little ring middle index thumb_bend thumb_rotate. 1000=open, 0=close, -1=hold"
    )
    parser.add_argument("--port", default="/dev/ttyUSB_robot_485")
    parser.add_argument("--id", type=int, required=True, help="left=1, right=2")
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--watch", type=float, default=2.0, help="seconds to print state after command")
    args = parser.parse_args()

    hand_id = args.id
    values = [clamp(v) for v in args.angles]

    print(f"Target hand id: {hand_id}")
    print("Target command:")
    for name, val in zip(FINGER_NAMES, values):
        print(f"  {name:13s}: {val}")

    if not args.yes:
        ans = input("\nSend this command? Type y to continue: ").strip().lower()
        if ans != "y":
            print("Canceled.")
            return

    with serial.Serial(args.port, BAUD, bytesize=8, parity="N", stopbits=1, timeout=0.3) as ser:
        print_state(ser, hand_id, "before")

        resp = write_register(ser, hand_id, ANGLE_SET, pack_six(values))
        print("\nwrite ack:", resp.hex(" "))

        start = time.time()
        while time.time() - start < args.watch:
            time.sleep(0.5)
            print_state(ser, hand_id, "after")

if __name__ == "__main__":
    main()