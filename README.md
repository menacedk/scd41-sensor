# scd41-sensor

CO2 / temperature / humidity (SCD41) and VOC / NOx (SGP41) sensor on a Wemos D1 mini Pro (ESP8266).

- Up to 3 WiFi profiles, each with its own MQTT broker (manual topics for Home Assistant)
- WiFiManager setup portal (`SCD41-Setup`), also opened automatically after 60 s without WiFi
- Web page at `http://scd41-stue.local` with live values, 1 h / 24 h averages, power-on time and uptime
- Status LED (built-in, GPIO2): fast even blink = AP mode, slower blink = WiFi but no MQTT,
  very slow blink = searching for WiFi, solid = CO2 ≥ 1300 ppm (off below 1200)

## Wiring

| Sensor | D1 mini |
|---|---|
| VDD | 3V3 |
| GND | G |
| SCL | D1 |
| SDA | D2 |

SCD41 (0x62) and SGP41 (0x59) share the I2C bus.

## Build

Board: `LOLIN(WEMOS) D1 mini Pro` (ESP8266 core 3.1.2). Libraries: Sensirion I2C SCD4x, Sensirion Core,
Sensirion I2C SGP41, Sensirion Gas Index Algorithm, WiFiManager (tzapu), PubSubClient.

```bash
arduino-cli compile --fqbn esp8266:esp8266:d1_mini_pro scd41_stue
```
