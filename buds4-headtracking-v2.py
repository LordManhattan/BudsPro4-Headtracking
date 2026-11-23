"""Galaxy Buds (2/3/4 Pro) head tracking for opentrack (Linux & Windows).

Usage:
  python3 buds2_headtracking.py AA:BB:CC:DD:EE:FF [options]

Controls while running:
  r = recenter (look straight ahead)
  p = pause/resume tracking (freeze camera for clicking cockpit instruments)
  u = calibrate up/pitch axis (look straight, press u, lift head within 3 s)
  Ctrl+C = quit

Global triggers on Linux (map to Stream Deck or GNOME shortcuts):
  Recenter: pkill -USR1 -f buds2_headtracking
  Pause:    pkill -USR2 -f buds2_headtracking
"""
import argparse
import ctypes
import json
import math
import os
import signal
import socket
import struct
import sys
import time
import uuid
from pathlib import Path

if os.name == "nt":
    import msvcrt

    def read_key():
        return msvcrt.getwch().lower() if msvcrt.kbhit() else None
else:
    import select
    import termios
    import tty

    def read_key():
        if not sys.stdin.isatty():
            return None
        return sys.stdin.read(1).lower() if select.select([sys.stdin], [], [], 0)[0] else None

# Management service of Buds+ and later models (GalaxyBudsClient: Uuids.SppNew)
SPP_UUID = uuid.UUID("2e73a4ad-332d-41fc-90e2-16bef06523f2")

SOM, EOM = 0xFD, 0xDD
SET_SPATIAL_AUDIO = 124
SPATIAL_AUDIO_DATA = 194
SPATIAL_AUDIO_CONTROL = 195

CTRL_ATTACH, CTRL_DETACH, CTRL_ATTACH_OK, CTRL_DETACH_OK, CTRL_KEEPALIVE = 0, 1, 2, 3, 4
DATA_GRV, DATA_WEAR_ON, DATA_WEAR_OFF, DATA_GYROCAL, DATA_STUCK = 32, 33, 34, 35, 36

KEEPALIVE_SEC = 2.0
PRINT_SEC = 0.1
RECONNECT_DELAY_SEC = 2.0
OPENTRACK_ADDR = ("127.0.0.1", 4242)

CAL_SEC = 3.0
AXES_FILE = Path(__file__).with_name("axes.json")

# In Samsung's GRV the world X axis is vertical (gravity).
WORLD_UP = (1.0, 0.0, 0.0)
# Default head-lift (pitch) axis in earbud coordinates; re-measured with 'u'.
LAT_AXIS = (0.99, 0.00, -0.13)

MOD_ALT, MOD_CONTROL, MOD_NOREPEAT, VK_R, WM_HOTKEY = 0x1, 0x2, 0x4000, 0x52, 0x0312


class OneEuroFilter:
    """Low-latency adaptive smoothing filter (steady when still, fast when moving)."""

    def __init__(self, min_cutoff=1.2, beta=0.03, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def reset(self):
        self.x_prev = None
        self.dx_prev = 0.0
        self.t_prev = None

    def __call__(self, x, t):
        if self.t_prev is None:
            self.x_prev = x
            self.t_prev = t
            return x
        dt = max(1e-4, t - self.t_prev)
        self.t_prev = t

        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev

        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode(msg_id, payload):
    body = bytes([msg_id, *payload])
    size = len(body) + 2
    return bytes([SOM]) + struct.pack("<H", size) + body + struct.pack("<H", crc16(body)) + bytes([EOM])


def frames(buf):
    """Extracts complete frames from buf: (msg_id, payload, raw frame, crc_ok)."""
    while True:
        start = buf.find(SOM)
        if start < 0:
            buf.clear()
            return
        del buf[:start]
        if len(buf) < 3:
            return
        size = struct.unpack_from("<H", buf, 1)[0] & 0x3FF
        total = 3 + size + 1
        if len(buf) < total:
            return
        frame = bytes(buf[:total])
        if size < 3 or frame[-1] != EOM:
            del buf[:1]
            continue
        del buf[:total]
        body = frame[3:1 + size]
        crc_ok = crc16(body) == struct.unpack_from("<H", frame, 1 + size)[0]
        yield body[0], body[1:], frame, crc_ok


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    x, y, z, w = q
    return -x, -y, -z, w


def q_angle_deg(a, b):
    """Angular distance in degrees between two unit quaternions."""
    dot = abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3])
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-9 else (0.0, 0.0, 0.0)


def rotate(v, q):
    x, y, z, _ = q_mul(q_mul(q, (*v, 0.0)), q_conj(q))
    return x, y, z


def cross(a, b):
    return a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]


def rotation_axis(d):
    """Axis of rotation d (oriented so angle < 180); None if rotated less than ~15 degrees."""
    x, y, z, w = d
    if w < 0:
        x, y, z = -x, -y, -z
    return normalize((x, y, z)) if math.sqrt(x * x + y * y + z * z) > 0.13 else None


def load_lat():
    try:
        lat = normalize(json.loads(AXES_FILE.read_text())["pitch"])
        if any(lat):
            return lat, True
    except (FileNotFoundError, KeyError, TypeError, json.JSONDecodeError):
        pass
    return normalize(LAT_AXIS), False


def save_lat(lat):
    AXES_FILE.write_text(json.dumps({"pitch": lat}, indent=2))


def head_angles(q, ref, lat):
    """Returns (yaw, pitch, roll) in degrees relative to ref:
    +yaw = left, +pitch = up, +roll = tilt head right.
    """
    up_ref = rotate(WORLD_UP, ref)
    nose_ear = normalize(cross(lat, up_ref))
    if not any(nose_ear):
        return 0.0, 0.0, 0.0
    left_ear = normalize(cross(up_ref, nose_ear))

    q_inv = q_conj(q)
    start = rotate(nose_ear, q_conj(ref))
    now = rotate(nose_ear, q_inv)

    yaw = math.atan2(start[1] * now[2] - start[2] * now[1],
                     start[1] * now[1] + start[2] * now[2])
    pitch = math.asin(max(-1.0, min(1.0, now[0])))

    left_now = rotate(left_ear, q_inv)
    up_now = rotate(up_ref, q_inv)
    roll = math.atan2(left_now[0], up_now[0])

    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def bar(yaw):
    pos = max(-20, min(20, round(-yaw / 3)))
    cells = ["-"] * 41
    cells[20] = "|"
    cells[20 + pos] = "#"
    return "L [" + "".join(cells) + "] R"


def sdp_rfcomm_channel(mac, service_uuid):
    pattern = b"\x35\x11\x1c" + service_uuid.bytes
    attrs = b"\x35\x05\x0a\x00\x00\xff\xff"
    params = pattern + b"\xff\xff" + attrs + b"\x00"
    with socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP) as sdp:
        sdp.settimeout(5.0)
        sdp.connect((mac, 1))
        sdp.send(struct.pack(">BHH", 0x06, 1, len(params)) + params)
        resp = sdp.recv(4096)
    i = resp.find(b"\x19\x00\x03\x08")
    if resp[:1] != b"\x07" or i < 0:
        raise OSError(f"SDP: {service_uuid} not found. Pass --channel manually (sdptool browse {mac}).")
    return resp[i + 4]


def connect(mac, channel=None):
    sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    sock.settimeout(5.0)
    if os.name != "nt" or channel:
        ch = channel or sdp_rfcomm_channel(mac, SPP_UUID)
        sock.connect((mac, ch))
        return sock
    ws2 = ctypes.WinDLL("ws2_32")
    ws2.connect.argtypes = [ctypes.c_size_t, ctypes.c_char_p, ctypes.c_int]
    addr = struct.pack("<HQ16sI", socket.AF_BLUETOOTH, int(mac.replace(":", ""), 16), SPP_UUID.bytes_le, 0)
    if ws2.connect(sock.fileno(), addr, len(addr)) != 0:
        err = ws2.WSAGetLastError()
        sock.close()
        raise OSError(err, f"Could not connect (WSA error {err}). Are the earbuds connected?")
    return sock


def setup_triggers():
    """Returns (recenter_hit(), pause_hit(), close(), help_text)."""
    script_name = Path(__file__).stem
    if os.name == "nt":
        from ctypes import wintypes
        user32 = ctypes.WinDLL("user32")
        ok = user32.RegisterHotKey(None, 1, MOD_CONTROL | MOD_ALT | MOD_NOREPEAT, VK_R)
        msg = wintypes.MSG()

        def recenter_hit():
            return bool(user32.PeekMessageW(ctypes.byref(msg), None, WM_HOTKEY, WM_HOTKEY, 1))

        def close():
            if ok:
                user32.UnregisterHotKey(None, 1)

        help_txt = "Ctrl+Alt+R = recenter" if ok else "Warning: Ctrl+Alt+R already taken"
        return recenter_hit, (lambda: False), close, help_txt

    rec_flag, pause_flag = [], []
    signal.signal(signal.SIGUSR1, lambda *_: rec_flag.append(1))
    signal.signal(signal.SIGUSR2, lambda *_: pause_flag.append(1))

    def recenter_hit():
        hit = bool(rec_flag)
        rec_flag.clear()
        return hit

    def pause_hit():
        hit = bool(pause_flag)
        pause_flag.clear()
        return hit

    help_txt = (f"pkill -USR1 -f {script_name} (recenter) | "
                f"pkill -USR2 -f {script_name} (pause/resume)")
    return recenter_hit, pause_hit, (lambda: None), help_txt


def main():
    parser = argparse.ArgumentParser(description="Galaxy Buds head tracking for opentrack / MSFS 2024")
    parser.add_argument("mac", help="Bluetooth MAC address, e.g. 8C:A3:EC:0C:FB:E3")
    parser.add_argument("--yaw-only", action="store_true", help="send yaw only (pitch and roll = 0)")
    parser.add_argument("--roll", type=float, default=0.0,
                        help="roll scale (0.0 = off [default], 0.4 = subtle, 1.0 = full 1:1)")
    parser.add_argument("--smooth", action="store_true",
                        help="enable lightweight One Euro smoothing (optional alongside opentrack Accela)")
    parser.add_argument("--max-jump", type=float, default=45.0,
                        help="reject single-packet orientation spikes above N degrees (default: 45, 0 = off)")
    parser.add_argument("--no-reconnect", action="store_true", help="exit instead of reconnecting on BT drop")
    parser.add_argument("--quiet", action="store_true", help="disable live terminal status line")
    parser.add_argument("--channel", type=int, help="RFCOMM channel (normally found automatically)")
    parser.add_argument("--trace", action="store_true", help="print every sent/received frame in hex")
    parser.add_argument("--seconds", type=float, default=0, help="quit after N seconds (0 = run forever)")
    args = parser.parse_args()

    lat, loaded_from_file = load_lat()
    recenter_hit, pause_hit, close_triggers, trigger_help = setup_triggers()

    term = None
    if os.name != "nt" and sys.stdin.isatty():
        term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    f_yaw = OneEuroFilter() if args.smooth else None
    f_pitch = OneEuroFilter() if args.smooth else None
    f_roll = OneEuroFilter() if args.smooth else None

    def reset_filters():
        if args.smooth:
            f_yaw.reset()
            f_pitch.reset()
            f_roll.reset()

    print(f"Axes: {'loaded ' + AXES_FILE.name if loaded_from_file else 'default (press u to calibrate for Buds 4 Pro)'}")
    print("Keys: r = recenter | p = pause | u = calibrate up | Ctrl+C = quit")
    print(f"Global: {trigger_help}")
    print(f"Target: UDP {OPENTRACK_ADDR[0]}:{OPENTRACK_ADDR[1]} "
          f"(roll={args.roll:g}, smooth={'on' if args.smooth else 'off'}, max-jump={args.max_jump:g} deg)")

    started = time.monotonic()
    try:
        while not args.seconds or time.monotonic() - started < args.seconds:
            sock = None
            try:
                sock = connect(args.mac, args.channel)
            except OSError as e:
                if args.no_reconnect:
                    print(f"\nConnection failed: {e}")
                    break
                print(f"\rWaiting for earbuds ({e})...          ", end="", flush=True)
                time.sleep(RECONNECT_DELAY_SEC)
                continue

            print("\nConnected to earbuds.")

            def send(msg_id, payload):
                frame = encode(msg_id, payload)
                if args.trace:
                    print(f"\n>> {msg_id:3d}  {frame.hex(' ')}")
                sock.sendall(frame)

            try:
                send(SET_SPATIAL_AUDIO, [1])
                send(SPATIAL_AUDIO_CONTROL, [CTRL_ATTACH])
                sock.settimeout(0.2)

                buf = bytearray()
                ref = last_q = None
                last_q_time = time.monotonic()
                consecutive_spikes = 0
                cal = None
                paused = False
                worn = True
                last_keepalive = time.monotonic()
                last_print = 0.0
                pkt_count = 0
                bad_crc = 0
                spikes = 0
                hz = 0.0
                hz_time = time.monotonic()

                while not args.seconds or time.monotonic() - started < args.seconds:
                    now = time.monotonic()
                    if now - last_keepalive >= KEEPALIVE_SEC:
                        send(SPATIAL_AUDIO_CONTROL, [CTRL_KEEPALIVE])
                        last_keepalive = now

                    if recenter_hit() and last_q:
                        ref = last_q
                        reset_filters()
                        print("\nRecentered.")

                    if pause_hit():
                        paused = not paused
                        print(f"\nTracking {'PAUSED' if paused else 'RESUMED'}.")

                    key = read_key()
                    if key:
                        if key == "r" and last_q:
                            ref = last_q
                            reset_filters()
                            print("\nRecentered.")
                        elif key == "p":
                            paused = not paused
                            print(f"\nTracking {'PAUSED' if paused else 'RESUMED'}.")
                        elif key == "u" and last_q:
                            ref = last_q
                            cal = {"until": now + CAL_SEC, "best": (0.0, 0.0, 0.0, 1.0)}
                            print(f"\nLook straight, then lift your head UP within {CAL_SEC:.0f}s...")

                    try:
                        chunk = sock.recv(1024)
                    except TimeoutError:
                        continue
                    if not chunk:
                        print("\nEarbuds closed the connection.")
                        break

                    buf += chunk
                    for msg_id, payload, frame, crc_ok in frames(buf):
                        if args.trace:
                            print(f"\n<< {msg_id:3d}  {frame.hex(' ')}{'' if crc_ok else '  (BAD CRC)'}")
                        if not crc_ok:
                            bad_crc += 1
                            continue

                        if msg_id == SPATIAL_AUDIO_CONTROL and payload:
                            msg = {CTRL_ATTACH_OK: "Sensor attached.",
                                   CTRL_DETACH_OK: "Sensor detached."}.get(payload[0], f"Control: {payload[0]}")
                            print(f"\n{msg}")
                        elif msg_id == SPATIAL_AUDIO_DATA and payload:
                            kind, data = payload[0], payload[1:]
                            if kind == DATA_GRV and len(data) >= 8:
                                raw = [v / 10000.0 for v in struct.unpack_from("<4h", data)]
                                norm = math.sqrt(sum(v * v for v in raw))
                                if not (0.5 < norm < 1.5):
                                    spikes += 1
                                    continue

                                q = tuple(v / norm for v in raw)
                                if last_q is not None and args.max_jump > 0:
                                    dt = max(0.001, now - last_q_time)
                                    allowed = max(args.max_jump, 1000.0 * dt)
                                    if q_angle_deg(q, last_q) > allowed:
                                        consecutive_spikes += 1
                                        if consecutive_spikes < 3:
                                            spikes += 1
                                            continue
                                consecutive_spikes = 0
                                last_q = q
                                last_q_time = now
                                ref = ref or last_q
                                pkt_count += 1

                                if now - hz_time >= 1.0:
                                    hz = pkt_count / (now - hz_time)
                                    pkt_count = 0
                                    hz_time = now

                                if cal:
                                    d = q_mul(last_q, q_conj(ref))
                                    if abs(d[3]) < abs(cal["best"][3]):
                                        cal["best"] = d
                                    if now >= cal["until"]:
                                        measured = rotation_axis(cal["best"])
                                        if measured is None:
                                            print("\nUp calibration failed: lift head at least 15 deg, press u again.")
                                        else:
                                            lat = measured
                                            save_lat(lat)
                                            print(f"\nSaved up axis to {AXES_FILE.name}: "
                                                  f"({lat[0]:+.2f}, {lat[1]:+.2f}, {lat[2]:+.2f})")
                                        cal = None

                                if paused or not worn:
                                    continue

                                yaw, pitch, roll = head_angles(last_q, ref, lat)
                                if args.yaw_only:
                                    pitch = 0.0
                                    roll = 0.0
                                else:
                                    roll *= args.roll

                                if args.smooth:
                                    yaw = f_yaw(yaw, now)
                                    pitch = f_pitch(pitch, now)
                                    roll = f_roll(roll, now)

                                # Opentrack UDP: x, y, z, yaw (+right), pitch (+up), roll (+right)
                                udp.sendto(struct.pack("<6d", 0.0, 0.0, 0.0, -yaw, pitch, roll), OPENTRACK_ADDR)

                                if not args.quiet and not args.trace and (now - last_print >= PRINT_SEC):
                                    print(f"\r{bar(yaw)}  Y:{yaw:+6.1f}  P:{pitch:+5.1f}  R:{roll:+5.1f}  "
                                          f"{hz:4.0f}Hz  CRC:{bad_crc}  Spk:{spikes}  ", end="", flush=True)
                                    last_print = now

                            elif kind == DATA_WEAR_ON:
                                worn = True
                                ref = None
                                last_q = None
                                consecutive_spikes = 0
                                reset_filters()
                                print("\nEarbuds in ears — auto-recentered.")
                            elif kind == DATA_WEAR_OFF:
                                worn = False
                                print("\nEarbuds taken out — tracking paused.")
                            elif kind == DATA_STUCK:
                                print("\nSensor stuck — re-attaching...")
                                send(SPATIAL_AUDIO_CONTROL, [CTRL_DETACH])
                                send(SPATIAL_AUDIO_CONTROL, [CTRL_ATTACH])
                            elif kind == DATA_GYROCAL:
                                print("\nGyro self-calibrated.")
                            elif args.trace:
                                print(f"\n   data type {kind}: {data.hex(' ')}")
            except OSError as e:
                print(f"\nBluetooth error: {e}")
            finally:
                if sock:
                    try:
                        send(SPATIAL_AUDIO_CONTROL, [CTRL_DETACH])
                        send(SET_SPATIAL_AUDIO, [0])
                    except OSError:
                        pass
                    sock.close()

            if args.no_reconnect:
                break
            time.sleep(RECONNECT_DELAY_SEC)

    except KeyboardInterrupt:
        pass
    finally:
        udp.close()
        close_triggers()
        if term:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, term)
        print("\nStopped.")


if __name__ == "__main__":
    main()
