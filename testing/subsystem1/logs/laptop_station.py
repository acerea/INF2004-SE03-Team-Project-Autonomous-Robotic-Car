#!/usr/bin/env python3
"""
laptop_station.py - laptop side of the picocar link.

- Decodes the car's binary frames (picocar-01/hb and picocar-01/tlm).
- Lets you type commands: start / stop / estop / reset / mode N

Install once:  pip install paho-mqtt
Run:           python laptop_station.py
"""
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
CAR = "picocar-01"
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
        counters = ""
        if len(f) >= 8:     # 8-byte heartbeat: cmd_ok at bit 41, cmd_bad at 49
            counters = (f" cmd_ok={get_bits(f, 41, 8)}"
                        f" cmd_bad={get_bits(f, 49, 8)}")
        note = "  <-- LAST WILL" if seq == 255 else ""
        return (f"HB seq={seq} ver={ver} link={LINK[link]} "
                f"uptime={up}s drops={drops}{counters}{note}")
    if typ == 1 and len(f) >= 4:
        st, obs = get_bits(f, 14, 3), get_bits(f, 17, 2)
        lost, speed = get_bits(f, 19, 1), get_bits(f, 20, 6)
        return (f"STATE seq={seq} state={VC_STATES[st]} "
                f"obstacle={OBS_STATES[obs]} link_lost={lost} "
                f"speed={speed}cm/s")
    if typ == 5 and len(f) >= 4:
        st, col, flt = get_bits(f, 14, 3), get_bits(f, 17, 1), get_bits(f, 18, 1)
        valid, mm, count = get_bits(f, 19, 1), get_bits(f, 20, 8), get_bits(f, 28, 4)
        hump = f"{mm}mm count={count}" if valid else "not measured"
        return (f"TERRAIN seq={seq} state={B4_STATES[st]} collision={col} "
                f"imu_fault={flt} hump={hump}")
    if typ == 6 and len(f) >= 4:
        st, cm = get_bits(f, 14, 2), get_bits(f, 16, 9)
        dist = "no echo" if cm == 0 else f"{cm}cm"
        return f"OBSTACLE seq={seq} state={OBS_STATES[st]} dist={dist}"
    if typ == 7 and len(f) >= 6:
        side, width = get_bits(f, 14, 2), get_bits(f, 16, 7)
        left, right = get_bits(f, 23, 9), get_bits(f, 32, 9)
        lt = "open" if left == 0 else f"{left}cm"
        rt = "open" if right == 0 else f"{right}cm"
        return (f"OBST_PROFILE seq={seq} side={SIDES[side]} width={width}cm "
                f"left={lt} right={rt}")
    return f"{topic} {name} seq={seq} ver={ver} raw={f.hex()}"


def on_connect(c, userdata, flags, rc, props=None):
    print("[station] connected to broker")
    c.subscribe(f"{CAR}/#")


def on_message(c, userdata, msg):
    if msg.topic == f"{CAR}/cmd":
        return
    print("[car]", decode(msg.topic, msg.payload))


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="laptop-station")
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