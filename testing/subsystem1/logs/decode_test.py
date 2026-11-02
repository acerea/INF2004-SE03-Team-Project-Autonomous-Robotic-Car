import paho.mqtt.client as mqtt

LINK = ["INIT","WIFI_CONNECTING","MQTT_CONNECTING","CONNECTED",
        "LOST","RECONNECTING","FAULT","OFFLINE"]

def get(buf, pos, n):
    val = 0
    for i in range(n):
        bit = (buf[(pos + i) // 8] >> ((pos + i) % 8)) & 1
        val |= bit << i
    return val

def on_message(c, u, msg):
    f = msg.payload
    if len(f) < 6:
        return
    seq, ver, typ = get(f, 0, 8), get(f, 8, 2), get(f, 10, 4)
    if typ == 0:
        link, up, drop = get(f, 14, 3), get(f, 17, 16), get(f, 33, 8)
        print(f"HB seq={seq} ver={ver} link={LINK[link]} uptime={up} drops={drop}")
    else:
        print(f"{msg.topic} seq={seq} type={typ} raw={f.hex()}")

c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
c.on_connect = lambda c, u, f, rc, p=None: c.subscribe("picocar-01/#")
c.on_message = on_message
c.connect("127.0.0.1", 1883)
c.loop_forever()