ESPHome SX126x driver.

SX126x component configures SX1261, SX1262 or SX1268 hardware for use in ESPHome. Tested with the Heltec WiFi LoRa 32 V3 Dev-board with SX1262.

To use this repo:

	external_components:
	  - source: github://swoboda1337/sx126x-esphome@main
	    components: [ sx126x ]

	sx126x:
	  dio0_pin: GPIO26
	  cs_pin: GPIO18
	  rst_pin: GPIO23
	  pa_pin: BOOST
	  pa_power: 4
	  bandwidth: 125_0kHz
	  crc_enable: true
	  frequency: 433920000
	  modulation: LORA
	  rx_start: true
	  sync_value: 0x33
	  preamble_size: 8
	  spreading_factor: 12
	  coding_rate: CR_4_6
	  on_packet:
	    then:
	      - lambda: |-
	          std::string message(x.begin(), x.end());
	          ESP_LOGD("lambda", "packet %.2f %.2f %s", rssi, snr, message.c_str());

	button:
	  - platform: template
	    name: "Template Button"
	    on_press:
	      then:
	        - sx126x.send_packet:
	           data: [0xC5, 0x51, 0x78, 0x82, 0xB7, 0xF9, 0x9C, 0x5C]
