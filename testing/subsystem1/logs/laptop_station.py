#!/usr/bin/env python3
"""
laptop_station.py - laptop side of the picocar link.

- Sends the laptop heartbeat the car's watchdog needs.
- Decodes the car's binary frames (picocar-01/hb and picocar-01/tlm).
- Lets you type commands: start / stop / estop / reset / mode N

Install once:  pip install paho-mqtt
Run:           python laptop_station.py
"""
import threading, time
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
CAR = "picocar-01"
HB_INTERVAL = 1.0
PROTO_VERSION = 1

LINK = ["INIT", "WIFI_CONNECTING", "MQTT_CONNECTING", "CONNECTED",
        "LOST", "RECONNECTING", "FAULT", "OFFLINE"]
TYPES = ["HEARTBEAT", "STATE", "MOTION", "LINE", "BARCODE", "TERRAIN", "OBSTACLE"]
CMDS = {"start": 0, "stop": 1, "estop": 2, "reset": 3, "mode": 4}


def get_bits(buf, pos, n):
    """Read n bits LSB-first starting at bit position pos."""
    val = 0
    for i in range(n):
        bit = (buf[(pos + i) // 8] >> ((pos + i) % 8)) & 1
        val |= bit << i
    return val


def put_bits(fields):
    """Pack (value, width) pairs LSB-first into bytes."""
    acc, pos = 0, 0
    for val, width in fields:
        acc |= (val & ((1 << width) - 1)) << pos
        pos += width
    return acc.to_bytes((pos + 7) // 8, "little")


def encode_cmd(seq, cmd, arg):
    # seq 8 | version 2 | command 3 | argument 4  -> 17 bits -> 3 bytes
    return put_bits([(seq, 8), (PROTO_VERSION, 2), (cmd, 3), (arg, 4)])


def decode(topic, f):
    if len(f) < 3:
        return f"{topic} short frame {f.hex()}"
    seq, ver, typ = get_bits(f, 0, 8), get_bits(f, 8, 2), get_bits(f, 10, 4)
    name = TYPES[typ] if typ < len(TYPES) else f"type{typ}"
    if typ == 0 and len(f) >= 6:
        link = get_bits(f, 14, 3)
        up = get_bits(f, 17, 16)
        drops = get_bits(f, 33, 8)
        note = "  <-- LAST WILL" if seq == 255 else ""
        return (f"HB seq={seq} ver={ver} link={LINK[link]} "
                f"uptime={up}s drops={drops}{note}")
    return f"{topic} {name} seq={seq} ver={ver} raw={f.hex()}"


def on_connect(c, userdata, flags, rc, props=None):
    print("[station] connected to broker")
    c.subscribe(f"{CAR}/#")


def on_message(c, userdata, msg):
    if msg.topic == f"{CAR}/cmd":
        return
    print("[car]", decode(msg.topic, msg.payload))


def heartbeat(c):
    while True:
        c.publish("picocar/laptop_heartbeat", "1")
        time.sleep(HB_INTERVAL)


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="laptop-station")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_start()
threading.Thread(target=heartbeat, args=(client,), daemon=True).start()

cmd_seq = 0
try:
    while True:
        line = input().strip().lower().split()
        if not line:
            continue
        if line[0] not in CMDS:
            print("commands: start | stop | estop | reset | mode N")
            continue
        arg = int(line[1]) if len(line) > 1 and line[1].isdigit() else 0
        frame = encode_cmd(cmd_seq, CMDS[line[0]], arg & 0xF)
        client.publish(f"{CAR}/cmd", frame, qos=1)
        print(f"[station] sent {line[0]} seq={cmd_seq} bytes={frame.hex()}")
        cmd_seq = (cmd_seq + 1) & 0xFF
except KeyboardInterrupt:
    pass