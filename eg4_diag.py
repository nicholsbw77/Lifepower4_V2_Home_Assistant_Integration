#!/usr/bin/env python3
"""
eg4_diag.py — Exhaustive diagnostic for EG4 LifePower4 V2 Modbus RTU.

Sweeps baud rates, parities, stop bits, and addresses to definitively
determine what configuration the batteries actually respond to.

Uses raw serial + manual CRC16 (NO pymodbus) to rule out library issues.
Run this ONCE to establish working parameters, then the main script
(eg4_modbus_mqtt.py) uses them.

Usage (on SA Pi):
    python3 /tmp/eg4_diag.py --port /dev/ttyUSB0
"""

import argparse
import serial
import time
import sys

BAUD_RATES   = [9600, 19200, 38400, 57600, 115200]
PARITIES     = [('N', serial.PARITY_NONE), ('E', serial.PARITY_EVEN), ('O', serial.PARITY_ODD)]
STOP_BITS    = [1, 2]
ADDRESSES    = list(range(1, 9))   # 1..8


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_read_request(addr: int, reg: int, count: int) -> bytes:
    req = bytes([addr, 0x03, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF])
    c = crc16(req)
    return req + bytes([c & 0xFF, c >> 8])


def validate_response(resp: bytes, addr: int, count: int) -> tuple:
    """Return (valid, parsed_registers_or_error_msg)."""
    expected_len = 5 + count * 2   # addr + func + bytecount + data + crc
    if len(resp) < 5:
        return False, f"short ({len(resp)} bytes)"
    if resp[0] != addr:
        return False, f"addr mismatch (got {resp[0]}, want {addr})"
    if resp[1] & 0x80:
        return False, f"modbus error {resp[2]:#x}"
    if resp[1] != 0x03:
        return False, f"func mismatch ({resp[1]:#x})"
    byte_count = resp[2]
    if byte_count != count * 2:
        return False, f"byte count {byte_count} != {count*2}"
    if len(resp) < 3 + byte_count + 2:
        return False, f"truncated ({len(resp)}/{3+byte_count+2})"
    data = resp[3:3 + byte_count]
    crc_got = resp[3 + byte_count] | (resp[3 + byte_count + 1] << 8)
    crc_calc = crc16(resp[:3 + byte_count])
    if crc_got != crc_calc:
        return False, f"crc mismatch ({crc_got:#x} vs {crc_calc:#x})"
    regs = [int.from_bytes(data[i:i+2], 'big') for i in range(0, byte_count, 2)]
    return True, regs


def try_config(port, baud, parity_char, parity_val, stop, addr, reg=0x0000, count=1, timeout=1.0):
    """Try one configuration. Return (success, info_dict_or_error)."""
    try:
        s = serial.Serial(
            port, baud,
            bytesize=8, parity=parity_val, stopbits=stop,
            timeout=timeout, exclusive=True,
        )
    except Exception as e:
        return False, f"open failed: {e}"

    try:
        time.sleep(0.05)   # let line settle
        s.reset_input_buffer()
        req = build_read_request(addr, reg, count)
        s.write(req)
        s.flush()
        # 3.5 char delay at baud, min 2ms
        time.sleep(max(0.002, 35.0 / baud))
        resp = s.read(5 + count * 2 + 8)   # generous read
        s.close()
    except Exception as e:
        s.close()
        return False, f"io error: {e}"

    if not resp:
        return False, "no response"

    valid, data = validate_response(resp, addr, count)
    if not valid:
        return False, f"bad response ({data}): {resp[:16].hex()}"

    return True, {
        "baud":   baud,
        "parity": parity_char,
        "stop":   stop,
        "addr":   addr,
        "regs":   data,
        "raw":    resp[:16].hex(),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--quick", action="store_true",
                    help="only test 9600/19200/115200 @ N81 to save time")
    args = ap.parse_args()

    print(f"EG4 LifePower4 V2 Modbus RTU Diagnostic")
    print(f"Port: {args.port}")
    print(f"{'='*70}\n")

    if args.quick:
        configs = [(b, 'N', serial.PARITY_NONE, 1) for b in [9600, 19200, 115200]]
    else:
        configs = [(b, pc, pv, sb)
                   for b in BAUD_RATES
                   for pc, pv in PARITIES
                   for sb in STOP_BITS]

    found = []
    total = len(configs) * len(ADDRESSES)
    done  = 0

    print(f"Sweeping {len(configs)} serial configs × {len(ADDRESSES)} addresses = {total} probes\n")

    # Pass 1: single-register read at reg 0x0000 to find any response
    for baud, pchar, pval, stop in configs:
        got_this_config = False
        for addr in ADDRESSES:
            done += 1
            ok, info = try_config(args.port, baud, pchar, pval, stop, addr,
                                  reg=0x0000, count=1, timeout=0.6)
            if ok:
                if not got_this_config:
                    print(f"\n{'─'*70}")
                    print(f"  {baud:>6} baud, {pchar}, {stop} stop")
                    print(f"{'─'*70}")
                    got_this_config = True
                print(f"    ✓ addr {addr}: reg[0x0000] = {info['regs'][0]} (0x{info['regs'][0]:04X})   raw: {info['raw']}")
                found.append((baud, pchar, pval, stop, addr))
            # Progress on same line every 10 probes
            if done % 20 == 0:
                sys.stdout.write(f"\r  progress: {done}/{total}   ")
                sys.stdout.flush()

    sys.stdout.write("\r" + " " * 50 + "\r")   # clear progress
    print(f"\n{'='*70}")
    print(f"SUMMARY: {len(found)} responding combinations\n")

    if not found:
        print("NO BATTERIES RESPONDED to any config.")
        print("")
        print("Things to check:")
        print("  1. Is SA in 'Use inverter values' mode?")
        print("     (If SA is actively polling in modbus/serial mode, our frames")
        print("      collide with SA's. Switch SA to 'Use inverter values'.)")
        print("  2. Is the adapter wired to the RS485-1 port (not COMM)?")
        print("  3. Is /dev/ttyUSB0 really the adapter for the batteries?")
        print("     Run: dmesg | grep tty    to confirm which device is which.")
        print("  4. Try --no-quick for full sweep including even/odd parity.")
        return 1

    # Pass 2: for each responding config, read 32 registers and show full dump
    print("Reading full 32-register block from each responder...\n")
    best = found[0]
    baud, pchar, pval, stop, _ = best
    # Deduplicate addresses per config
    by_config = {}
    for b, pc, pv, sb, a in found:
        by_config.setdefault((b, pc, pv, sb), []).append(a)

    for (b, pc, pv, sb), addrs in by_config.items():
        print(f"\n{'='*70}")
        print(f"Config: {b} baud {pc}{sb}")
        print(f"{'='*70}")
        for addr in addrs:
            ok, info = try_config(args.port, b, pc, pv, sb, addr,
                                  reg=0x0000, count=32, timeout=1.5)
            if not ok:
                print(f"  Addr {addr}: second read failed ({info})")
                continue
            regs = info['regs']
            pack_mv_raw = regs[0]
            # signed16 for current
            cur_raw = regs[1] - 0x10000 if regs[1] > 0x7FFF else regs[1]
            cells = regs[2:18]
            temps = [r - 0x10000 if r > 0x7FFF else r for r in regs[18:21]]
            soc   = regs[21] if len(regs) > 21 else None
            soh   = regs[23] if len(regs) > 23 else None

            print(f"\n  ── Addr {addr} ─────────────────────────────────")
            print(f"    Pack voltage  (0x0000): {pack_mv_raw * 0.01:.2f} V  (raw {pack_mv_raw})")
            print(f"    Current       (0x0001): {cur_raw * 0.01:+.2f} A  (raw {cur_raw})")
            cells_valid = [c for c in cells if 2000 < c < 4500]
            print(f"    Cell voltages (0x0002-0x0011): {len(cells_valid)} valid, "
                  f"min={min(cells_valid) if cells_valid else 0}mV, "
                  f"max={max(cells_valid) if cells_valid else 0}mV, "
                  f"delta={max(cells_valid)-min(cells_valid) if cells_valid else 0}mV")
            for i, mv in enumerate(cells, 1):
                mark = "" if 2000 < mv < 4500 else "  <- out of range"
                print(f"       Cell {i:2d}: {mv} mV{mark}")
            print(f"    Temps (0x0012-0x0014): {temps} °C")
            print(f"    SOC   (0x0015):         {soc}%")
            print(f"    SOH   (0x0017):         {soh}%")

    print(f"\n{'='*70}")
    print("RECOMMENDED CONFIG for eg4_modbus_mqtt.py:")
    best = found[0]
    print(f"    BAUDRATE   = {best[0]}")
    print(f"    PARITY     = '{best[1]}'")
    print(f"    STOPBITS   = {best[3]}")
    print(f"    ADDRESSES  = {sorted(set(f[4] for f in found))}")
    print(f"{'='*70}")


if __name__ == "__main__":
    sys.exit(main() or 0)
