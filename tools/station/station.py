#!/usr/bin/env python3
"""
station.py - laptop side of the picocar link, with a browser control panel.

Does everything laptop_station.py does (decode the car's binary frames,
republish them as JSON on <car>/view/<kind>, 0.5 s keep-alive PING), and
also takes commands from control_panel.html:

    browser --JSON--> <car>/ui/cmd --> station.py --3-byte frame--> <car>/cmd

The browser never talks to the car directly, so there is still exactly ONE
sender of command sequence numbers (the car rejects old/duplicate ones).
Do not run laptop_station.py at the same time.

UI command JSON (topic <car>/ui/cmd):
    {"cmd": "start" | "stop" | "estop" | "reset"}
    {"cmd": "calibrate"}               -> MODE 0 (IR recalibration)
    {"cmd": "speed", "mm_s": 100}      -> MODE n, n = cruise in cm/s (1-15)
Every command sent is echoed as JSON on <car>/ui/ack for the event log.

Install once:  pip install paho-mqtt
Run:           python tools/station/station.py [--broker 127.0.0.1] [--car picocar-01]
Then open tools/station/control_panel.html in a browser.
"""
import argparse
import json
import os
import threading
import time

import paho.mqtt.client as mqtt

PROTO_VERSION = 1
PING_S = 0.5            # the car stops after 2 s without any command

LINK = ["INIT", "WIFI_CONNECTING", "MQTT_CONNECTING", "CONNECTED",
        "LOST", "RECONNECTING", "FAULT", "OFFLINE"]
TYPES = ["HEARTBEAT", "STATE", "MOTION", "LINE", "BARCODE", "TERRAIN",
         "OBSTACLE", "OBST_PROFILE"]
B4_STATES = ["UNKNOWN", "STATIONARY", "MOVING", "TURNING", "CLIMBING",
             "DESCENDING", "LIFTED", "FAULT"]
OBS_STATES = ["CLEAR", "SLOWDOWN", "EMERGENCY", "?"]
SIDES = ["NONE", "LEFT", "RIGHT", "?"]
VC_STATES = ["IDLE", "FOLLOWING", "TURNING", "BYPASSING", "BLOCKED",
             "STOPPED", "FAULT", "CALIBRATING"]
MO_MODES = ["STOP", "SPEED", "DISTANCE", "TURN"]
BC_CMDS = ["NONE", "LEFT", "RIGHT", "STRAIGHT", "U-TURN", "?", "?", "?"]
CMDS = {"start": 0, "stop": 1, "estop": 2, "reset": 3, "mode": 4, "ping": 5}
CRUISE_MIN_CM_S, CRUISE_MAX_CM_S = 1, 15     # MODE argument is 4 bits
IRCAL_KINDS = ["standing", "sweep", "sweep cancelled", "?"]
# VC_EVT_xxx in vehicle.c -> readable text ({arg} is filled in)
EVENTS = {
    1: "laptop link lost ({arg} ms without a command), stopping",
    2: "line lost, searching {dir}",
    3: "line found again",
    4: "line not found, holding stopped (START again on the line)",
    5: "robot overturned, motors stopped",
    6: "robot lifted, motors stopped",
    7: "obstacle during barcode move, stopping",
    8: "bypass {side}, sideways {lat} cm",
    9: "bypass forward leg {arg} cm",
    10: "START ignored: robot lifted or overturned",
    11: "motion request busy (cmd {arg}), retrying",
    12: "sweep calibration started",
    13: "calibration turn timed out (step {arg}), old levels kept",
    14: "calibration refused: {why}",
    15: "standing calibration (FAULT: no sweep)",
    16: "IMU fault: lift/overturn stop unavailable",
    17: "calibration done, middle sensor on the line",
    18: "calibration done, but the line is NOT under the middle sensor",
}


def get_bits(buf, pos, n):
    val = 0
    for i in range(n):
        val |= ((buf[(pos + i) // 8] >> ((pos + i) % 8)) & 1) << i
    return val


def get_signed(buf, pos, n):
    return get_bits(buf, pos, n) - (1 << (n - 1))


def put_bits(fields):
    acc, pos = 0, 0
    for val, width in fields:
        acc |= (val & ((1 << width) - 1)) << pos
        pos += width
    return acc.to_bytes((pos + 7) // 8, "little")


def encode_cmd(seq, cmd, arg):
    # seq 8 | version 2 | command 3 | argument 4 -> 3 bytes
    return put_bits([(seq, 8), (PROTO_VERSION, 2), (cmd, 3), (arg, 4)])


def decode(f, car):
    """One frame -> (kind, dict), or None if it is too short."""
    if len(f) < 2:
        return None
    seq, typ = get_bits(f, 0, 8), get_bits(f, 10, 4)
    d = {"car": car, "seq": seq}
    if typ == 0 and len(f) >= 6:
        d.update(link=LINK[get_bits(f, 14, 3)], uptime_s=get_bits(f, 17, 16),
                 drops=get_bits(f, 33, 8))
        if len(f) >= 8:
            d.update(cmd_ok=get_bits(f, 41, 8), cmd_bad=get_bits(f, 49, 8))
        d["last_will"] = (seq == 255)
        return "hb", d
    if typ == 1 and len(f) >= 4:
        d.update(state=VC_STATES[get_bits(f, 14, 3)],
                 obstacle=OBS_STATES[get_bits(f, 17, 2)],
                 link_lost=get_bits(f, 19, 1), speed_cm_s=get_bits(f, 20, 6),
                 # 0 = firmware without the cruise field (older build)
                 cruise_cm_s=get_bits(f, 26, 4))
        return "state", d
    if typ == 2 and len(f) >= 8:
        d.update(mode=MO_MODES[get_bits(f, 14, 2)], target_cm_s=get_bits(f, 16, 6),
                 left_cm_s=get_signed(f, 22, 11) / 10,
                 right_cm_s=get_signed(f, 33, 11) / 10,
                 left_pwm=get_signed(f, 44, 8), right_pwm=get_signed(f, 52, 8))
        return "motion", d
    if typ == 3 and len(f) >= 4:
        d.update(state=["CALI", "TRACK", "BARCODE", "LINE_LOST"][get_bits(f, 14, 2)],
                 sensors=get_bits(f, 16, 3),
                 junction=["NONE", "LEFT", "RIGHT", "BOTH"][get_bits(f, 19, 2)],
                 line_pos=get_bits(f, 21, 11) - 1024)
        return "line", d
    if typ == 4 and len(f) >= 3:
        code = get_bits(f, 14, 5)
        d.update(letter=chr(ord("A") + code) if code < 26 else "?",
                 command=BC_CMDS[get_bits(f, 19, 3)])
        return "barcode", d
    if typ == 5 and len(f) >= 4:
        d.update(motion=B4_STATES[get_bits(f, 14, 3)],
                 overturned=get_bits(f, 17, 1), imu_fault=get_bits(f, 18, 1))
        return "terrain", d
    if typ == 6 and len(f) >= 4:
        cm = get_bits(f, 16, 9)
        d.update(state=OBS_STATES[get_bits(f, 14, 2)],
                 cm=cm, fault=(cm == 511))
        return "obstacle", d
    if typ == 7 and len(f) >= 6:
        d.update(side=SIDES[get_bits(f, 14, 2)], width_cm=get_bits(f, 16, 7),
                 left_cm=get_bits(f, 23, 9), right_cm=get_bits(f, 32, 9))
        return "obst_profile", d
    if typ == 8 and len(f) >= 12:
        d["kind"] = IRCAL_KINDS[get_bits(f, 14, 2)]
        pos = 16
        for name in ("L", "C", "R"):
            d[name] = {"white": get_bits(f, pos, 12), "black": get_bits(f, pos + 12, 12),
                       "new": bool(get_bits(f, pos + 24, 1))}
            pos += 25
        return "ircal", d
    if typ == 9 and len(f) >= 5:
        code, arg = get_bits(f, 14, 8), get_bits(f, 22, 16)
        text = EVENTS.get(code, f"event {code} arg {arg}")
        d.update(code=code, arg=arg, text=text.format(
            arg=arg, dir="right" if arg else "left",
            side="left" if (arg >> 8) == 1 else "right", lat=arg & 0xFF,
            why="lifted or overturned" if arg else "car is driving"))
        return "event", d
    d.update(type=TYPES[typ] if typ < len(TYPES) else f"type{typ}", raw=f.hex())
    return "other", d


class Station:
    def __init__(self, broker, port, car, log_dir):
        self.car = car
        self.lock = threading.Lock()
        self.log_lock = threading.Lock()
        # One log file per run: every decoded frame, command and ack, with a
        # local timestamp. Readable without the serial monitor.
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(
            log_dir, time.strftime(f"station_{car}_%Y%m%d_%H%M%S.log"))
        self.log_fh = open(self.log_path, "a", encoding="utf-8", buffering=1)
        print(f"[station] logging to {self.log_path}")
        # Continue the sequence across restarts (the car rejects old numbers).
        # Same file as laptop_station.py, so switching between them is safe.
        self.seq_file = os.path.join(os.path.expanduser("~"), ".picocar_cmd_seq")
        try:
            with open(self.seq_file) as fh:
                self.seq = (int(fh.read().strip()) + 1) & 0xFF
        except (OSError, ValueError):
            self.seq = 0
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id=f"station-{car}")
        self.c.on_connect = self.on_connect
        self.c.on_message = self.on_message
        self.c.connect(broker, port, 60)

    def log(self, kind, d):
        stamp = time.strftime("%H:%M:%S") + ".%03d" % (int(time.time() * 1000) % 1000)
        rest = " ".join(f"{k}={v}" for k, v in d.items()
                        if k not in ("car", "t", "raw"))
        with self.log_lock:
            self.log_fh.write(f"[{stamp}] {kind:<12} {rest}\n")

    # -- MQTT -------------------------------------------------------------
    def on_connect(self, c, userdata, flags, rc, props=None):
        print(f"[station] broker connected, car '{self.car}'")
        self.log("station", {"event": "broker connected"})
        for t in ("hb", "tlm", "ui/cmd"):
            c.subscribe(f"{self.car}/{t}")

    def on_message(self, c, userdata, msg):
        if msg.topic.endswith("/ui/cmd"):
            self.ui_command(msg.payload)
            return
        res = decode(msg.payload, self.car)
        if res is None:
            return
        kind, d = res
        d["t"] = time.time()
        c.publish(f"{self.car}/view/{kind}", json.dumps(d))
        self.log(kind, d)
        if kind not in ("motion", "line", "obstacle"):     # keep the terminal calm
            print(f"[car] {kind} {d}")

    # -- commands ---------------------------------------------------------
    def send(self, name, arg=0, quiet=False):
        with self.lock:
            frame = encode_cmd(self.seq, CMDS[name], arg & 0xF)
            self.c.publish(f"{self.car}/cmd", frame, qos=1)
            sent = self.seq
            self.seq = (self.seq + 1) & 0xFF
            try:
                with open(self.seq_file, "w") as fh:
                    fh.write(str(sent))
            except OSError:
                pass
        if not quiet:
            print(f"[station] sent {name} arg={arg} seq={sent}")
            self.log("cmd", {"sent": name, "arg": arg, "seq": sent})
        return sent

    def ack(self, **kw):
        kw["t"] = time.time()
        self.c.publish(f"{self.car}/ui/ack", json.dumps(kw))

    def ui_command(self, payload):
        try:
            req = json.loads(payload)
            name = str(req.get("cmd", "")).lower()
        except (ValueError, AttributeError):
            self.ack(ok=False, error="bad JSON")
            return
        if name in ("start", "stop", "estop", "reset"):
            self.ack(ok=True, cmd=name, seq=self.send(name))
        elif name == "calibrate":
            self.ack(ok=True, cmd="calibrate", seq=self.send("mode", 0))
        elif name == "speed":
            try:
                cm = round(float(req.get("mm_s")) / 10.0)
            except (TypeError, ValueError):
                self.ack(ok=False, cmd="speed", error="no mm_s value")
                return
            cm = max(CRUISE_MIN_CM_S, min(CRUISE_MAX_CM_S, cm))
            self.ack(ok=True, cmd="speed", mm_s=cm * 10, seq=self.send("mode", cm))
        else:
            self.ack(ok=False, cmd=name, error="unknown command")

    def run(self):
        self.c.loop_start()

        def pinger():
            n = 0
            while True:
                time.sleep(PING_S)
                self.send("ping", quiet=True)
                n += 1
                if n % 2 == 0:      # lets the panel see that station.py runs
                    self.c.publish(f"{self.car}/ui/alive", str(int(time.time())))

        threading.Thread(target=pinger, daemon=True).start()
        print("[station] keep-alive on: closing this window stops the car within 2 s")
        print("[station] typed commands also work: start | stop | estop | reset | cal | speed <mm/s>")
        try:
            while True:
                words = input().strip().lower().split()
                if not words:
                    continue
                if words[0] in ("cal", "calibrate"):
                    self.ui_command(json.dumps({"cmd": "calibrate"}))
                elif words[0] == "speed" and len(words) > 1:
                    self.ui_command(json.dumps({"cmd": "speed", "mm_s": words[1]}))
                else:
                    self.ui_command(json.dumps({"cmd": words[0]}))
        except EOFError:            # no terminal (started in the background)
            try:
                while True:
                    time.sleep(3600)
            except KeyboardInterrupt:
                pass
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--car", default="picocar-01", help="MQTT_TOPIC_BASE of the car")
    ap.add_argument("--log-dir", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "logs"),
        help="where to write station_*.log (default: <repo>/logs)")
    a = ap.parse_args()
    Station(a.broker, a.port, a.car, os.path.normpath(a.log_dir)).run()
