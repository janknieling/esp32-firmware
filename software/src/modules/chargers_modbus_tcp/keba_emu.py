#!/usr/bin/env python3
"""KEBA P30 Modbus TCP emulator for testing the chargers_modbus_tcp module.

Emulates the register set documented in the "KeContact P30 Modbus TCP
Programmers Guide V1.07" including its quirks:
- unit id must be 255
- only function codes 3 (read holding registers) and 6 (write single register)
- at most one value (two registers) per read request
- readable values are UINT32 in two big-endian registers

A simple vehicle simulation is included: plug/unplug the cable and start or
stop charging via keyboard commands.

Usage: keba_emu.py [--port 502] [--phases 3] [--x-series]
"""

import argparse
import socket
import struct
import sys
import threading
import time

parser = argparse.ArgumentParser()
parser.add_argument("--port", type=int, default=502)
parser.add_argument("--phases", type=int, default=3, choices=[1, 3])
parser.add_argument("--x-series", action="store_true", help="support phase switching via register 5052")
parser.add_argument("--serial", type=int, default=18416854)
args = parser.parse_args()


class KebaState:
    def __init__(self):
        self.lock = threading.Lock()
        self.plugged = False        # cable plugged into vehicle
        self.car_wants_charge = False
        self.enabled = True         # register 5014
        self.set_current = 6000     # register 5004 [mA]
        self.hw_current = 32000     # register 1110 [mA]
        self.failsafe_current = 0   # register 5016
        self.failsafe_timeout = 0   # register 5018
        self.failsafe_deadline = None
        self.phase_switch_source = 0
        self.phases = args.phases   # register 1552
        self.total_energy = 12345678  # 0.1 Wh
        self.session_energy = 0       # 0.1 Wh
        self.last_write = time.monotonic()

    def touch_failsafe(self):
        if self.failsafe_timeout > 0:
            self.failsafe_deadline = time.monotonic() + self.failsafe_timeout

    def failsafe_active(self):
        return self.failsafe_deadline is not None and time.monotonic() > self.failsafe_deadline

    def charging_state(self):
        # 0 startup, 1 not ready, 2 ready, 3 charging, 4 error, 5 suspended
        if not self.enabled or self.failsafe_active() and self.failsafe_current == 0:
            return 5 if self.plugged else 1
        if not self.plugged:
            return 1
        if self.car_wants_charge:
            return 3
        return 2

    def cable_state(self):
        return 7 if self.plugged else 0

    def current(self, phase):
        if self.charging_state() != 3:
            return 0
        if phase > self.phases:
            return 0
        limit = self.set_current
        if self.failsafe_active() and self.failsafe_current > 0:
            limit = self.failsafe_current
        return min(limit, self.hw_current)

    def power(self):
        # mW
        return sum(self.current(p) for p in (1, 2, 3)) * 230

    def read(self, addr):
        s = self
        values = {
            1000: s.charging_state(),
            1004: s.cable_state(),
            1006: 0,
            1008: s.current(1),
            1010: s.current(2),
            1012: s.current(3),
            1014: args.serial,
            1016: 304111,
            1018: 0x30A1B00,
            1020: s.power() // 1000,
            1036: s.total_energy,
            1040: 230 if s.charging_state() == 3 else 0,
            1042: 230 if s.charging_state() == 3 and s.phases == 3 else 0,
            1044: 230 if s.charging_state() == 3 and s.phases == 3 else 0,
            1046: 928,
            1100: min(s.set_current, s.hw_current),
            1110: s.hw_current,
            1500: 0,
            1502: s.session_energy,
            1550: s.phase_switch_source,
            1552: s.phases,
            1600: s.failsafe_current,
            1602: s.failsafe_timeout,
        }
        return values.get(addr)

    def write(self, addr, value):
        s = self
        s.last_write = time.monotonic()
        s.touch_failsafe()

        if addr == 5004:
            if not 6000 <= value <= 63000:
                return False
            s.set_current = value
        elif addr == 5010:
            pass  # set energy limit, not simulated
        elif addr == 5012:
            pass  # unlock plug, not simulated
        elif addr == 5014:
            s.enabled = value != 0
        elif addr == 5016:
            s.failsafe_current = value
        elif addr == 5018:
            s.failsafe_timeout = value
            s.failsafe_deadline = time.monotonic() + value if value > 0 else None
        elif addr == 5020:
            pass  # failsafe persist, not simulated
        elif addr == 5050:
            s.phase_switch_source = value
        elif addr == 5052:
            if not args.x_series or s.phase_switch_source != 3:
                return False
            s.phases = 3 if value else 1
        else:
            return False

        print(f"WRITE {addr} = {value} -> state={s.charging_state()} enabled={s.enabled} "
              f"current={s.set_current} phases={s.phases} failsafe={s.failsafe_current}/{s.failsafe_timeout}")
        return True


state = KebaState()


def handle_request(data):
    if len(data) < 12:
        return None

    tid, pid, length, uid, fc = struct.unpack(">HHHBB", data[:8])

    def exception(code):
        return struct.pack(">HHHBBB", tid, 0, 3, uid, fc | 0x80, code)

    if uid != 255:
        return exception(0x0B)  # gateway target failed; KEBA wants unit id 255

    if fc == 3:
        addr, count = struct.unpack(">HH", data[8:12])
        if count > 2:
            return exception(0x03)  # KEBA: max read length is one UINT32 value

        value = state.read(addr & ~1)
        if value is None:
            return exception(0x02)

        regs = struct.pack(">I", value & 0xFFFFFFFF)
        payload = regs[:count * 2] if (addr % 2) == 0 else regs[2:2 + count * 2]
        return struct.pack(">HHHBBB", tid, 0, 2 + 1 + len(payload), uid, fc, len(payload)) + payload

    if fc == 6:
        addr, value = struct.unpack(">HH", data[8:12])
        with state.lock:
            if not state.write(addr, value):
                return exception(0x02)
        return struct.pack(">HHHBB", tid, 0, 6, uid, fc) + data[8:12]

    return exception(0x01)


def client_thread(conn, peer):
    print(f"client connected: {peer}")
    try:
        while True:
            header = conn.recv(8)
            if len(header) < 8:
                break
            length = struct.unpack(">H", header[4:6])[0]
            body = conn.recv(length - 2) if length > 2 else b""
            response = handle_request(header + body)
            if response is not None:
                conn.sendall(response)
    except OSError:
        pass
    finally:
        conn.close()
        print(f"client disconnected: {peer}")


def console_thread():
    print("commands: p = plug/unplug vehicle, c = car wants/stops charging, q = quit")
    for line in sys.stdin:
        cmd = line.strip().lower()
        with state.lock:
            if cmd == "p":
                state.plugged = not state.plugged
                if not state.plugged:
                    state.car_wants_charge = False
                    state.session_energy = 0
                print(f"plugged: {state.plugged}")
            elif cmd == "c":
                state.car_wants_charge = not state.car_wants_charge
                print(f"car wants charge: {state.car_wants_charge}")
            elif cmd == "q":
                sys.exit(0)


def energy_thread():
    while True:
        time.sleep(1)
        with state.lock:
            if state.charging_state() == 3:
                wh = state.power() / 1000 / 3600  # W -> Wh per second
                state.total_energy += int(wh * 10)
                state.session_energy += int(wh * 10)


threading.Thread(target=console_thread, daemon=True).start()
threading.Thread(target=energy_thread, daemon=True).start()

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(("0.0.0.0", args.port))
server.listen(4)
print(f"KEBA P30 emulator listening on port {args.port} (phases={args.phases}, x-series={args.x_series})")

while True:
    conn, peer = server.accept()
    threading.Thread(target=client_thread, args=(conn, peer), daemon=True).start()
