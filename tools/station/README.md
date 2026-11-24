# Ground station (control panel)

`station.py` + `control_panel.html`: drive and debug the car from a browser.

## Run

1. Mosquitto needs a WebSocket listener as well as the normal port. In `mosquitto.conf`:

   ```
   listener 1883
   allow_anonymous true
   listener 9001
   protocol websockets
   ```

2. `pip install paho-mqtt`, then
   `python tools/station/station.py --broker 127.0.0.1 --car picocar-01`
   (`--car` = `MQTT_TOPIC_BASE` in `config/wifi_secrets.h`).
3. Open `tools/station/control_panel.html` in a browser (it loads mqtt.js
   from unpkg, so it needs internet the first time).

Run **either** `station.py` **or** `testing/subsystem1/logs/laptop_station.py`,
not both: each one sends command sequence numbers and keep-alive PINGs, and
two senders confuse the car's replay check. They share `~/.picocar_cmd_seq`,
so switching between them is safe.

## What it does

| Panel                     | Car side                                             |
|---------------------------|------------------------------------------------------|
| START / STOP / RESET / E-STOP | `S1_CMD_START` / `STOP` / `RESET` / `ESTOP`       |
| CALIBRATE IR              | `S1_CMD_MODE`, arg 0 -> `ir_request_calibration()`; refused while driving |
| CRUISE SPEED (mm/s)       | `S1_CMD_MODE`, arg 1-15 = cruise in cm/s (default 10 = 100 mm/s) |
| "car reports" speed       | new 4-bit `cruise` field at the end of the STATE frame |
| Event log                 | built from the decoded frames + command acks; SAVE .LOG downloads it |

Keys: SPACE = STOP, ESC = E-STOP (when not typing in a field).

Topics added: `<car>/ui/cmd` (panel -> station, JSON), `<car>/ui/ack`
(station -> panel, JSON), `<car>/ui/alive` (station heartbeat). The car's
own topics (`/tlm`, `/hb`, `/cmd`) and frame formats are unchanged except
for the extra STATE field, which older decoders ignore.
