#!/usr/bin/env python3
"""
laptop_station.py - laptop side of the picocar link.

- Decodes the car's binary frames (picocar-01/hb and picocar-01/tlm) and
  prints them.
- Republishes every decoded frame as readable JSON on picocar-01/view/<kind>
  (state, hb, terrain, obstacle, obst_profile, ...), so an MQTT client such
  as MQTT Explorer shows the car's data with its sequence number and time.
  The car itself only sends binary (minimum-bit change requirement).
- Lets you type commands: start / stop / estop / reset / mode N

Install once:  pip install paho-mqtt
Run:           python laptop_station.py
"""
import json
import time
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
CAR = "picocar-01"
VIEW = f"{CAR}/view"
PROTO_VERSION = 1

LINK = ["INIT", "WIFI_CONNECTING", "MQTT_CONNECTING", "CONNECTED",
        "LOST", "RECONNECTING", "FAULT", "OFFLINE"]
TYPES = ["HEARTBEAT", "STATE", "MOTION", "LINE", "BARCODE", "TERRAIN",
         "OBSTACLE", "OBST_PROFILE"]
B4_STATES = ["UNKNOWN", "STATIONARY", "MOVING", "TURNING", "CLIMBING",
             "DESCENDING", "IMPACT", "FAULT"]
OBS_STATES = ["CLEAR", "SLOWDOWN", "EMERGENCY", "?"]
SIDES = ["NONE", "LEFT", "RIGHT", "?"]
VC_STATES = ["IDLE", "FOLLOWING", "TURNING", "BYPASSING", "BLOCKED",
             "STOPPED", "FAULT", "?"]
BC_CMDS = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "?", "?", "?"]
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


def cm_text(cm, zero):
    return zero if cm == 0 else f"{cm}cm"


def decode_fields(f):
    """Decode one frame into (kind, fields), or None if it is too short."""
    if len(f) < 2:
        return None
    seq, ver, typ = get_bits(f, 0, 8), get_bits(f, 8, 2), get_bits(f, 10, 4)
    d = {"car": CAR, "seq": seq, "ver": ver}
    if typ == 0 and len(f) >= 6:
        d.update(link=LINK[get_bits(f, 14, 3)], uptime_s=get_bits(f, 17, 16),
                 drops=get_bits(f, 33, 8))
        if len(f) >= 8:     # 8-byte heartbeat: cmd_ok at bit 41, cmd_bad at 49
            d.update(cmd_ok=get_bits(f, 41, 8), cmd_bad=get_bits(f, 49, 8))
        if seq == 255:
            d["last_will"] = True
        return "hb", d
    if typ == 1 and len(f) >= 4:
        d.update(state=VC_STATES[get_bits(f, 14, 3)],
                 obstacle=OBS_STATES[get_bits(f, 17, 2)],
                 link_lost=get_bits(f, 19, 1), speed_cm_s=get_bits(f, 20, 6))
        return "state", d
    if typ == 4 and len(f) >= 3:
        code = get_bits(f, 14, 5)
        d.update(letter=chr(ord("A") + code) if code < 26 else "unreadable",
                 command=BC_CMDS[get_bits(f, 19, 3)])
        return "barcode", d
    if typ == 5 and len(f) >= 4:
        valid = get_bits(f, 19, 1)
        d.update(motion=B4_STATES[get_bits(f, 14, 3)],
                 collision=get_bits(f, 17, 1), imu_fault=get_bits(f, 18, 1))
        if valid:
            d.update(hump_mm=get_bits(f, 20, 8), hump_count=get_bits(f, 28, 4))
        else:
            d["hump"] = "not measured"
        return "terrain", d
    if typ == 6 and len(f) >= 4:
        d.update(state=OBS_STATES[get_bits(f, 14, 2)],
                 distance=cm_text(get_bits(f, 16, 9), "no echo"))
        return "obstacle", d
    if typ == 7 and len(f) >= 6:
        d.update(side=SIDES[get_bits(f, 14, 2)],
                 width=f"{get_bits(f, 16, 7)}cm",
                 left=cm_text(get_bits(f, 23, 9), "open"),
                 right=cm_text(get_bits(f, 32, 9), "open"))
        return "obst_profile", d
    name = TYPES[typ] if typ < len(TYPES) else f"type{typ}"
    d.update(type=name, raw=f.hex())
    return name.lower(), d


def as_text(kind, d):
    """One readable line for the terminal."""
    rest = " ".join(f"{k}={v}" for k, v in d.items()
                    if k not in ("car", "ver", "time", "raw"))
    note = "  <-- LAST WILL" if d.get("last_will") else ""
    return f"{kind.upper()} {rest}{note}"


def on_connect(c, userdata, flags, rc, props=None):
    print("[station] connected to broker")
    c.subscribe(f"{CAR}/hb")
    c.subscribe(f"{CAR}/tlm")


def on_message(c, userdata, msg):
    res = decode_fields(msg.payload)
    if res is None:
        print(f"[car] {msg.topic} short frame {msg.payload.hex()}")
        return
    kind, d = res
    d["time"] = time.strftime("%H:%M:%S")
    d["raw"] = msg.payload.hex()
    print("[car]", as_text(kind, d))
    c.publish(f"{VIEW}/{kind}", json.dumps(d))


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                     client_id="laptop-station")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, 1883, 60)
client.loop_start()

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
