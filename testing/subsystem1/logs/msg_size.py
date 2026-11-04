#!/usr/bin/env python3
"""
msg_size.py - message size comparison for the minimum-bit change requirement.

Compares subsystem 1's binary MQTT payloads with an equivalent text encoding
of exactly the same fields (key=value and JSON), using real heartbeats from a
capture file (one "<laptop ms> <hex payload>" line per message, as recorded
for S1-M07). Commands are compared over every valid command value.

Sizes are reported for the payload alone and for the whole MQTT 3.1.1
PUBLISH packet (fixed header, topic, packet id, payload).

Run: python3 msg_size.py <capture.txt>
"""
import json
import re
import sys

TOPIC_HB = "picocar-01/hb"
TOPIC_CMD = "picocar-01/cmd"
QOS_HB = 1
QOS_CMD = 1
CMD_BYTES = 3


def get_bits(buf, pos, n):
    """Read n bits LSB-first starting at bit position pos."""
    val = 0
    for i in range(n):
        val |= ((buf[(pos + i) // 8] >> ((pos + i) % 8)) & 1) << i
    return val


def decode_hb(f):
    return {"seq": get_bits(f, 0, 8), "ver": get_bits(f, 8, 2),
            "type": get_bits(f, 10, 4), "link": get_bits(f, 14, 3),
            "up": get_bits(f, 17, 16), "drops": get_bits(f, 33, 8),
            "ok": get_bits(f, 41, 8), "bad": get_bits(f, 49, 8)}


def as_kv(d):
    return " ".join(f"{k}={v}" for k, v in d.items())


def as_json(d):
    return json.dumps(d, separators=(",", ":"))


def publish_bytes(topic, payload_len, qos):
    """Size of an MQTT 3.1.1 PUBLISH packet."""
    rem = 2 + len(topic) + (2 if qos else 0) + payload_len
    rem_len_bytes = 1 if rem < 128 else 2
    return 1 + rem_len_bytes + rem


def stats(sizes):
    return min(sizes), sum(sizes) / len(sizes), max(sizes)


def row(name, sizes, topic, qos):
    lo, avg, hi = stats(sizes)
    plo, pavg, phi = stats([publish_bytes(topic, s, qos) for s in sizes])
    print(f"  {name:<10} payload {lo:3d} / {avg:5.1f} / {hi:3d} B"
          f"   MQTT packet {plo:3d} / {pavg:5.1f} / {phi:3d} B")


def main():
    text = open(sys.argv[1], encoding="utf-8", errors="ignore").read()
    frames = [bytes.fromhex(h) for _, h in
              re.findall(r"(\d{13}) ([0-9a-f]+)", text)]
    hbs = [decode_hb(f) for f in frames if f[0] != 0xFF]   # skip Last-Will
    if not hbs:
        sys.exit("no heartbeats found")

    print(f"Heartbeat ({len(hbs)} real frames from {sys.argv[1]})"
          "  [min / avg / max]")
    row("binary", [len(f) for f in frames if f[0] != 0xFF], TOPIC_HB, QOS_HB)
    row("key=value", [len(as_kv(d)) for d in hbs], TOPIC_HB, QOS_HB)
    row("JSON", [len(as_json(d)) for d in hbs], TOPIC_HB, QOS_HB)
    print("  example key=value:", as_kv(hbs[-1]))
    print("  example JSON:     ", as_json(hbs[-1]))

    cmds = [{"seq": s, "ver": 1, "cmd": c, "arg": a}
            for s in range(256) for c in range(5) for a in range(16)]
    print(f"\nCommand (all {len(cmds)} valid seq/cmd/arg values)"
          "  [min / avg / max]")
    row("binary", [CMD_BYTES] * len(cmds), TOPIC_CMD, QOS_CMD)
    row("key=value", [len(as_kv(d)) for d in cmds], TOPIC_CMD, QOS_CMD)
    row("JSON", [len(as_json(d)) for d in cmds], TOPIC_CMD, QOS_CMD)


if __name__ == "__main__":
    main()
