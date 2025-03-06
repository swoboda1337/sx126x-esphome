SX126x component configures SX1261, SX1262 or SX1268 hardware for use in ESPHome. Tested with the Heltec WiFi LoRa 32 V3 Dev-board with SX1262.

To use this repo (example for Heltec V3) with LoRa:

	external_components:
	  - source: github://swoboda1337/sx126x-esphome@main
	    components: [ sx126x ]

	spi:
	  clk_pin: GPIO9
	  mosi_pin: GPIO10
	  miso_pin: GPIO11

	sx126x:
	  id: sx126x_id
	  dio1_pin: GPIO14
	  cs_pin: GPIO8
	  busy_pin: GPIO13
	  rst_pin: GPIO12
	  pa_power: 3
	  bandwidth: 125_0kHz
	  crc_enable: true
	  frequency: 433920000
	  modulation: LORA
	  rx_start: true
	  # payload_length: 8
	  hw_version: sx1262
	  rf_switch: true
	  sync_value: [0x14, 0x24]
	  preamble_size: 8
	  spreading_factor: 7
	  coding_rate: CR_4_6
	  tcxo_voltage: 1_8V
	  tcxo_delay: 5ms
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

To use this repo (example for Heltec V3) with FSK:

	external_components:
	  - source: github://swoboda1337/sx126x-esphome@main
	    components: [ sx126x ]

	spi:
	  clk_pin: GPIO9
	  mosi_pin: GPIO10
	  miso_pin: GPIO11

	sx126x:
	  id: sx126x_id
	  dio1_pin: GPIO14
	  cs_pin: GPIO8
	  busy_pin: GPIO13
	  rst_pin: GPIO12
	  pa_power: 3
	  bandwidth: 78_2kHz
	  crc_enable: true
	  frequency: 433920000
	  modulation: FSK
	  rx_start: true
	  payload_length: 8
	  hw_version: sx1262
	  bitrate: 4800
	  rf_switch: true
	  sync_value: [0x33, 0x33]
	  preamble_size: 4
	  preamble_detect: 2
	  tcxo_voltage: 1_8V
	  tcxo_delay: 5ms
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
