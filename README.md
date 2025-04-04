ESPHome SX126x driver.

SX126x component configures SX1261, SX1262 or SX1268 hardware for use in ESPHome. Tested with the Heltec WiFi LoRa 32 V3 Dev-board with SX1262.

Docs can be found here:

https://deploy-preview-4794--esphome.netlify.app/components/sx126x

ESPHome PR:

https://github.com/esphome/esphome/pull/8516

To use the PR:

	external_components:
	  - source: github://pr#8516
	    components: [ sx126x ]

To use this repo:

	external_components:
	  - source: github://swoboda1337/sx126x-esphome@main
	    components: [ sx126x ]
