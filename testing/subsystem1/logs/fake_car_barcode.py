#!/usr/bin/env python3
"""
fake_car_barcode.py - TEST ONLY. Pretends to be the car and publishes barcode
frames (type 4, same binary format as subsystem 3 will send) to
picocar-01/tlm, to check laptop_station.py and the MQTT Explorer view before
subsystem 3 publishes real ones. Never run it during the demo.

Run while laptop_station.py is running:  python fake_car_barcode.py
"""
import time
import paho.mqtt.client as mqtt

CAR = "picocar-01"
PROTO_VERSION = 1
TYPE_BARCODE = 4
CMDS = {"NONE": 0, "LEFT": 1, "RIGHT": 2, "STRAIGHT": 3, "U-TURN": 4}
TESTS = [("A", "LEFT"), ("B", "RIGHT"), ("C", "STRAIGHT"), ("D", "U-TURN"),
         (None, "NONE")]          # None = unreadable barcode


def put_bits(fields):
    """Pack (value, width) pairs LSB-first into bytes."""
    acc, pos = 0, 0
    for val, width in fields:
        acc |= (val & ((1 << width) - 1)) << pos
        pos += width
    return acc.to_bytes((pos + 7) // 8, "little")


def barcode_frame(seq, letter, cmd):
    code = ord(letter) - ord("A") if letter else 31
    return put_bits([(seq, 8), (PROTO_VERSION, 2), (TYPE_BARCODE, 4),
                     (code, 5), (CMDS[cmd], 3)])


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-car")
client.connect("127.0.0.1", 1883, 60)
client.loop_start()
for seq, (letter, cmd) in enumerate(TESTS):
    frame = barcode_frame(seq, letter, cmd)
    client.publish(f"{CAR}/tlm", frame, qos=1).wait_for_publish()
    print(f"[fake car] sent barcode {letter or 'unreadable'} {cmd} "
          f"bytes={frame.hex()}")
    time.sleep(1)
client.loop_stop()
client.disconnect()
