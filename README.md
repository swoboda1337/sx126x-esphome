SX126x component configures SX1261, SX1262 or SX1268 hardware for use in ESPHome. Tested with the Heltec WiFi LoRa 32 V3 and SX1262.

This has now been merged into ESPHome 2025.7, docs can be found here:

https://deploy-preview-4794--esphome.netlify.app/components/sx126x

To use with a prevous version of ESPHome:
```
external_components:
  - source: github://pr#8516
    components: [ sx126x ]
```
