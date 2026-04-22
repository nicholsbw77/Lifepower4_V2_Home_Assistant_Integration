#!/usr/bin/env python3
"""
eg4_modbus_mqtt.py — EG4 LifePower4 V2 Modbus RTU → MQTT poller

Polls all EG4 batteries on the RS485-1 bus via Modbus RTU and publishes
per-cell voltages, temperatures, current, pack voltage, SOC, SOH, and
cycle count to Home Assistant via MQTT Discovery.

Runs on the Solar Assistant Pi (192.168.50.56) using /dev/ttyUSB0.
SA must be in "Use inverter values" battery mode so it doesn't compete
on the bus. Our script becomes the sole Modbus master.

Register map (EG4 LifePower4 V2, confirmed from live capture):
    0x0000   uint16   Pack voltage × 10mV
    0x0001   int16    Current × 10mA (negative = charging)
    0x0002-0x0011  uint16[16]  Cell voltages (mV, 16 cells)
    0x0012-0x0014  int16[3]    Temperatures (°C)
    0x0015   uint16   SOC (%)
    0x0017   uint16   SOH (%)
    0x0069+  ASCII    Model string "LFP-51.2V100Ah-V1.0..."

Uses raw serial + manual CRC16 (no pymodbus) because:
  1. Sidesteps any library-version quirks
  2. Fine control over inter-frame timing (critical for RS485 half-duplex)
  3. No hidden retry/reconnect logic

Deploy:
    scp eg4_modbus_mqtt.py solar-assistant@192.168.50.56:/home/solar-assistant/
    # test manually first:
    python3 /home/solar-assistant/eg4_modbus_mqtt.py
    # then install as service (see systemd block at bottom)
"""

import json
import logging
import sys
import time
import argparse

import serial
import paho.mqtt.client as mqtt


# ─────────────────────────────────────────────────────────────────────────────
# Configuration — edit to match results from eg4_diag.py
# ─────────────────────────────────────────────────────────────────────────────

SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE    = 9600      # eg4_diag.py will confirm; 9600 is the typical default
PARITY      = serial.PARITY_NONE
STOPBITS    = 1
BYTESIZE    = 8

MQTT_BROKER = "x.x.x.x"
MQTT_PORT   = 1883
MQTT_USER   = "user"
MQTT_PASS   = "password"
MQTT_PREFIX = "eg4_name"
MQTT_RETAIN = True
MQTT_QOS    = 1

# Batteries to poll — auto-scan at startup if list is empty
BATTERY_ADDRS = []   # [] = scan addresses 1..8; override with fixed list if desired
SCAN_RANGE    = range(1, 9)

POLL_INTERVAL     = 5.0    # seconds between full poll cycles
INTER_FRAME_GAP   = 0.05   # seconds between requests to same bus (Modbus 3.5 char rule)
RESPONSE_TIMEOUT  = 1.0    # per-request timeout
MAX_RETRIES       = 2      # retries per failed register read
DISCOVERY_EVERY   = 60     # re-publish HA Discovery every N cycles

# Register map
REG_BLOCK_START = 0x0000
REG_BLOCK_COUNT = 32        # reads 0x0000..0x001F in one shot
REG_PACK_MV     = 0x0000    # × 10mV
REG_CURRENT     = 0x0001    # × 10mA, int16
REG_CELL_START  = 0x0002
REG_CELL_COUNT  = 16        # 0x0002..0x0011
REG_TEMP_START  = 0x0012
REG_TEMP_COUNT  = 3
REG_SOC         = 0x0015
REG_SOH         = 0x0017

# Data validation bounds
CELL_MV_MIN = 2000          # anything below this is "cell not present"
CELL_MV_MAX = 4500
TEMP_C_MIN  = -40
TEMP_C_MAX  = 100
CURRENT_A_MIN = -300
CURRENT_A_MAX =  300


# ─────────────────────────────────────────────────────────────────────────────
# Logging
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger("eg4_modbus")


# ─────────────────────────────────────────────────────────────────────────────
# Modbus RTU — raw, no library
# ─────────────────────────────────────────────────────────────────────────────

def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _signed16(val: int) -> int:
    return val - 0x10000 if val > 0x7FFF else val


class ModbusRTU:
    def __init__(self, port, baud, parity, stopbits, bytesize):
        self.ser = serial.Serial(
            port, baud, bytesize=bytesize, parity=parity, stopbits=stopbits,
            timeout=RESPONSE_TIMEOUT, write_timeout=RESPONSE_TIMEOUT,
            exclusive=True,
        )
        self.baud = baud
        # 3.5-char idle at baud rate, min 2ms
        self.idle_time = max(0.002, 35.0 / baud)
        log.info("Serial open: %s @ %d %s%d",
                 port, baud, parity, stopbits)

    def close(self):
        self.ser.close()

    def read_holding(self, addr: int, reg: int, count: int,
                     retries: int = MAX_RETRIES) -> list:
        """Read `count` holding registers starting at `reg` from slave `addr`.
        Returns list of uint16 values, or raises RuntimeError on failure."""
        req = bytes([addr, 0x03, reg >> 8, reg & 0xFF, count >> 8, count & 0xFF])
        c = _crc16(req)
        req += bytes([c & 0xFF, c >> 8])

        last_err = None
        for attempt in range(retries + 1):
            try:
                time.sleep(self.idle_time)
                self.ser.reset_input_buffer()
                self.ser.write(req)
                self.ser.flush()
                time.sleep(self.idle_time)

                resp = self.ser.read(5 + count * 2)
                if not resp:
                    last_err = "no response"
                    continue

                if len(resp) < 5:
                    last_err = f"short response ({len(resp)} bytes): {resp.hex()}"
                    continue
                if resp[0] != addr:
                    last_err = f"address mismatch: got {resp[0]}, want {addr}"
                    continue
                if resp[1] & 0x80:
                    last_err = f"modbus exception code {resp[2]:#x}"
                    continue
                if resp[1] != 0x03:
                    last_err = f"function mismatch: {resp[1]:#x}"
                    continue

                byte_count = resp[2]
                if byte_count != count * 2:
                    last_err = f"byte count {byte_count} != {count * 2}"
                    continue

                # Some adapters deliver CRC bytes slightly late — read them if short
                total_needed = 3 + byte_count + 2
                if len(resp) < total_needed:
                    more = self.ser.read(total_needed - len(resp))
                    resp += more

                if len(resp) < total_needed:
                    last_err = f"CRC bytes missing: got {len(resp)}/{total_needed}"
                    continue

                crc_got = resp[3 + byte_count] | (resp[3 + byte_count + 1] << 8)
                crc_calc = _crc16(resp[:3 + byte_count])
                if crc_got != crc_calc:
                    last_err = f"CRC mismatch: got {crc_got:#x} calc {crc_calc:#x}"
                    continue

                data = resp[3:3 + byte_count]
                regs = [int.from_bytes(data[i:i + 2], 'big')
                        for i in range(0, byte_count, 2)]
                return regs

            except serial.SerialException as e:
                last_err = f"serial error: {e}"

        raise RuntimeError(f"addr={addr} reg=0x{reg:04X} count={count}: {last_err}")

    def ping(self, addr: int) -> bool:
        """Quick probe of one address. Returns True if it responded valid."""
        try:
            self.read_holding(addr, 0x0000, 1, retries=1)
            return True
        except Exception:
            return False

    def scan(self, addrs) -> list:
        found = []
        for a in addrs:
            if self.ping(a):
                log.info("  addr %d: responding", a)
                found.append(a)
            else:
                log.debug("  addr %d: silent", a)
        return found


# ─────────────────────────────────────────────────────────────────────────────
# Battery data parser
# ─────────────────────────────────────────────────────────────────────────────

def parse_battery(regs: list, addr: int) -> dict:
    """Parse a 32-register read (starting at 0x0000) into a battery data dict.
    Returns empty dict if the data fails validation."""
    if len(regs) < 24:
        log.warning("Pack %d: register read too short (%d < 24)", addr, len(regs))
        return {}

    pack_mv_raw = regs[REG_PACK_MV]            # × 10mV
    current_raw = _signed16(regs[REG_CURRENT]) # × 10mA

    pack_v      = pack_mv_raw * 0.01           # volts
    current_a   = current_raw * 0.01           # amps
    power_w     = round(pack_v * current_a, 1)

    # Sanity checks
    if not (30 <= pack_v <= 70):
        log.warning("Pack %d: implausible pack voltage %.2fV — skipping", addr, pack_v)
        return {}
    if not (CURRENT_A_MIN <= current_a <= CURRENT_A_MAX):
        log.warning("Pack %d: implausible current %.1fA — clamping to None", addr, current_a)
        current_a = None
        power_w   = None

    # Cell voltages
    cell_mv_raw = regs[REG_CELL_START - REG_BLOCK_START :
                       REG_CELL_START - REG_BLOCK_START + REG_CELL_COUNT]
    cell_mv = [mv for mv in cell_mv_raw if CELL_MV_MIN <= mv <= CELL_MV_MAX]
    if not cell_mv:
        log.warning("Pack %d: no valid cell voltages", addr)
        return {}

    # Temperatures
    temp_raw = regs[REG_TEMP_START - REG_BLOCK_START :
                    REG_TEMP_START - REG_BLOCK_START + REG_TEMP_COUNT]
    temps_c = []
    for t in temp_raw:
        t_s = _signed16(t)
        if TEMP_C_MIN <= t_s <= TEMP_C_MAX:
            temps_c.append(t_s)

    soc = regs[REG_SOC - REG_BLOCK_START] if (REG_SOC - REG_BLOCK_START) < len(regs) else None
    soh = regs[REG_SOH - REG_BLOCK_START] if (REG_SOH - REG_BLOCK_START) < len(regs) else None

    if soc is not None:
        soc = max(0, min(100, soc))
    if soh is not None:
        soh = max(0, min(100, soh))

    return {
        "pack_num":        addr,
        "pack_voltage_v":  round(pack_v, 3),
        "pack_voltage_mv": pack_mv_raw * 10,
        "current_a":       round(current_a, 2) if current_a is not None else None,
        "power_w":         power_w,
        "cell_count":      len(cell_mv),
        "cell_mv":         cell_mv,
        "cell_max_mv":     max(cell_mv),
        "cell_min_mv":     min(cell_mv),
        "cell_avg_mv":     round(sum(cell_mv) / len(cell_mv), 1),
        "cell_delta_mv":   max(cell_mv) - min(cell_mv),
        "temps_c":         temps_c,
        "soc":             soc,
        "soh":             soh,
    }


# ─────────────────────────────────────────────────────────────────────────────
# MQTT
# ─────────────────────────────────────────────────────────────────────────────

def build_mqtt() -> mqtt.Client:
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)

    def on_connect(client, userdata, flags, rc, props):
        if rc == 0:
            log.info("MQTT connected → %s:%d", MQTT_BROKER, MQTT_PORT)
        else:
            log.error("MQTT connect failed rc=%s", rc)

    def on_disconnect(client, userdata, flags, rc, props):
        log.warning("MQTT disconnected rc=%s", rc)

    c.on_connect    = on_connect
    c.on_disconnect = on_disconnect
    c.reconnect_delay_set(min_delay=2, max_delay=30)
    c.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=60)
    c.loop_start()
    return c


def publish_discovery(mqttc: mqtt.Client, d: dict):
    pack = d["pack_num"]
    device = {
        "identifiers":  [f"eg4_bms_pack_{pack}"],
        "name":         f"EG4 Battery Pack {pack}",
        "model":        "EG4 LifePower4 LFP-51.2V 100Ah V2",
        "manufacturer": "EG4",
    }

    scalars = [
        ("pack_voltage_v",  "Pack Voltage",       "V",   "voltage",     "measurement",      "mdi:flash"),
        ("current_a",       "Current",            "A",   "current",     "measurement",      "mdi:current-dc"),
        ("power_w",         "Power",              "W",   "power",       "measurement",      "mdi:lightning-bolt"),
        ("soc",             "State of Charge",    "%",   "battery",     "measurement",      None),
        ("soh",             "State of Health",    "%",   None,          "measurement",      "mdi:battery-heart"),
        ("cell_count",      "Cell Count",         None,  None,          None,               "mdi:counter"),
        ("cell_max_mv",     "Cell Voltage Max",   "mV",  "voltage",     "measurement",      "mdi:battery-arrow-up"),
        ("cell_min_mv",     "Cell Voltage Min",   "mV",  "voltage",     "measurement",      "mdi:battery-arrow-down"),
        ("cell_avg_mv",     "Cell Voltage Avg",   "mV",  "voltage",     "measurement",      "mdi:battery"),
        ("cell_delta_mv",   "Cell Voltage Delta", "mV",  "voltage",     "measurement",      "mdi:delta"),
        ("pack_voltage_mv", "Pack Voltage mV",    "mV",  "voltage",     "measurement",      None),
    ]

    for field, name, unit, dc, sc, icon in scalars:
        uid = f"eg4_bms_pack_{pack}_{field}"
        cfg = {
            "name":        name,
            "unique_id":   uid,
            "state_topic": f"{MQTT_PREFIX}/pack_{pack}/{field}",
            "device":      device,
        }
        if unit: cfg["unit_of_measurement"] = unit
        if dc:   cfg["device_class"]        = dc
        if sc:   cfg["state_class"]         = sc
        if icon: cfg["icon"]                = icon
        mqttc.publish(f"homeassistant/sensor/{uid}/config",
                      json.dumps(cfg), qos=MQTT_QOS, retain=True)

    for i in range(1, d["cell_count"] + 1):
        uid = f"eg4_bms_pack_{pack}_cell_{i:02d}_mv"
        cfg = {
            "name":                f"Cell {i:02d} Voltage",
            "unique_id":           uid,
            "state_topic":         f"{MQTT_PREFIX}/pack_{pack}/cell_{i:02d}_mv",
            "unit_of_measurement": "mV",
            "device_class":        "voltage",
            "state_class":         "measurement",
            "device":              device,
        }
        mqttc.publish(f"homeassistant/sensor/{uid}/config",
                      json.dumps(cfg), qos=MQTT_QOS, retain=True)

    for i in range(1, len(d["temps_c"]) + 1):
        uid = f"eg4_bms_pack_{pack}_temp_{i:02d}_c"
        cfg = {
            "name":                f"Temp Sensor {i:02d}",
            "unique_id":           uid,
            "state_topic":         f"{MQTT_PREFIX}/pack_{pack}/temp_{i:02d}_c",
            "unit_of_measurement": "°C",
            "device_class":        "temperature",
            "state_class":         "measurement",
            "device":              device,
        }
        mqttc.publish(f"homeassistant/sensor/{uid}/config",
                      json.dumps(cfg), qos=MQTT_QOS, retain=True)

    log.info("Discovery published for pack_%d (%d cells, %d temps)",
             pack, d["cell_count"], len(d["temps_c"]))


def publish_data(mqttc: mqtt.Client, d: dict):
    pack = d["pack_num"]
    base = f"{MQTT_PREFIX}/pack_{pack}"

    for k in ("pack_voltage_v", "pack_voltage_mv",
              "cell_count", "cell_max_mv", "cell_min_mv",
              "cell_avg_mv", "cell_delta_mv"):
        mqttc.publish(f"{base}/{k}", str(d[k]), qos=MQTT_QOS, retain=MQTT_RETAIN)

    if d.get("current_a") is not None:
        mqttc.publish(f"{base}/current_a", str(d["current_a"]), qos=MQTT_QOS, retain=MQTT_RETAIN)
        mqttc.publish(f"{base}/power_w",   str(d["power_w"]),   qos=MQTT_QOS, retain=MQTT_RETAIN)

    if d.get("soc") is not None:
        mqttc.publish(f"{base}/soc", str(d["soc"]), qos=MQTT_QOS, retain=MQTT_RETAIN)
    if d.get("soh") is not None:
        mqttc.publish(f"{base}/soh", str(d["soh"]), qos=MQTT_QOS, retain=MQTT_RETAIN)

    for i, mv in enumerate(d["cell_mv"], 1):
        mqttc.publish(f"{base}/cell_{i:02d}_mv", str(mv),
                      qos=MQTT_QOS, retain=MQTT_RETAIN)

    for i, t in enumerate(d["temps_c"], 1):
        mqttc.publish(f"{base}/temp_{i:02d}_c", str(t),
                      qos=MQTT_QOS, retain=MQTT_RETAIN)

    i_str = f"{d['current_a']:+.1f}A" if d.get("current_a") is not None else "?"
    log.info("Pack %d  V=%.2fV  I=%s  SOC=%s%%  cells=%d  delta=%dmV",
             pack, d["pack_voltage_v"], i_str,
             d.get("soc"), d["cell_count"], d["cell_delta_mv"])


# ─────────────────────────────────────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",  default=SERIAL_PORT)
    ap.add_argument("--baud",  type=int, default=BAUDRATE)
    ap.add_argument("--addrs", default="",
                    help="comma-separated addresses; empty = auto-scan")
    ap.add_argument("--once",  action="store_true",
                    help="poll once and exit (for testing)")
    ap.add_argument("--no-mqtt", action="store_true",
                    help="skip MQTT (for local testing)")
    args = ap.parse_args()

    log.info("EG4 Modbus MQTT poller — port=%s baud=%d broker=%s",
             args.port, args.baud, MQTT_BROKER)

    bus = ModbusRTU(args.port, args.baud, PARITY, STOPBITS, BYTESIZE)

    # Determine addresses
    if args.addrs:
        active = [int(a) for a in args.addrs.split(",")]
        log.info("Using fixed addresses: %s", active)
    elif BATTERY_ADDRS:
        active = BATTERY_ADDRS
        log.info("Using configured addresses: %s", active)
    else:
        log.info("Scanning addresses %s..%s for responses",
                 SCAN_RANGE.start, SCAN_RANGE.stop - 1)
        active = bus.scan(SCAN_RANGE)
        if not active:
            log.error("No batteries responded. Check:")
            log.error("  1. SA is in 'Use inverter values' battery mode (not actively polling)")
            log.error("  2. /dev/ttyUSB0 is the EG4 RS485-1 adapter")
            log.error("  3. Baud rate is correct (run eg4_diag.py to sweep)")
            sys.exit(1)
        log.info("Found responders: %s", active)

    mqttc = None if args.no_mqtt else build_mqtt()
    if mqttc:
        time.sleep(1.0)

    cycle          = 0
    discovery_sent = set()

    while True:
        cycle_start = time.monotonic()

        for addr in active:
            try:
                regs = bus.read_holding(addr, REG_BLOCK_START, REG_BLOCK_COUNT)
            except Exception as e:
                log.warning("Pack %d read failed: %s", addr, e)
                continue

            d = parse_battery(regs, addr)
            if not d:
                continue

            if mqttc:
                if addr not in discovery_sent or cycle % DISCOVERY_EVERY == 0:
                    publish_discovery(mqttc, d)
                    discovery_sent.add(addr)
                publish_data(mqttc, d)
            else:
                log.info("Pack %d: %s", addr, d)

        cycle += 1
        if args.once:
            break

        elapsed = time.monotonic() - cycle_start
        time.sleep(max(0.0, POLL_INTERVAL - elapsed))

    bus.close()
    if mqttc:
        mqttc.loop_stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Stopped by user")

# ─────────────────────────────────────────────────────────────────────────────
# /etc/systemd/system/eg4-modbus-mqtt.service:
#
# [Unit]
# Description=EG4 Modbus RS485 → MQTT poller
# After=network.target
#
# [Service]
# Type=simple
# User=solar-assistant
# WorkingDirectory=/home/solar-assistant
# ExecStart=/home/solar-assistant/.venv/bin/python3 /home/solar-assistant/eg4_modbus_mqtt.py
# Restart=always
# RestartSec=10
# StandardOutput=journal
# StandardError=journal
# SupplementaryGroups=dialout
#
# [Install]
# WantedBy=multi-user.target
# ─────────────────────────────────────────────────────────────────────────────
