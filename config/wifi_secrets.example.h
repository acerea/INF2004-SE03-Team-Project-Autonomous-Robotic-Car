/*
 *	wifi_secrets.example.h
 *
 *	Copy this file to wifi_secrets.h (same directory) and fill in your own
 *	network's credentials. wifi_secrets.h is listed in .gitignore -- it
 *	will never be committed, so it's safe to put your real password there.
 *
 *	Only used when built with NET=lwip (see build_make/pico_rp2040.mk and
 *	lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c).
 */

#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

#define WIFI_SSID     "your-network-name"
#define WIFI_PASSWORD "your-network-password"

/* Only used when built with MQTT=1 (also requires NET=lwip).
 * Broker: Mosquitto on the laptop. 192.168.137.1 is the laptop's own
 * address when it runs the Windows mobile hotspot.
 * Client ID: the broker allows one connection per ID, so two Picos with
 * the same ID keep disconnecting each other. Use your own ID when testing
 * on your own Pico at the same time as someone else.
 * Topics: MQTT_TOPIC_BASE/tlm, /hb and /cmd (see mqtt_utk.c). */
#define MQTT_BROKER_IP   "192.168.137.1"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "picocar-01"
#define MQTT_TOPIC_BASE  "picocar-01"

#endif /* WIFI_SECRETS_H */
