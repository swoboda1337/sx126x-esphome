Warning this is a WIP, only a few hours of work has been put into it. Code was ported from multiple sources, it is messy and needs to be cleaned up. Basic rx and tx does work for LoRa but more testing is needed. FSK has not been added yet.

SX126x component configures SX1261, SX1262 or SX1268 hardware for use in ESPHome. Tested with the Heltec WiFi LoRa 32 V3 Dev-board with SX1262.

To use this repo (example for Heltec V3):

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
	  sync_value: 0x12
	  preamble_size: 8
	  spreading_factor: 7
	  coding_rate: CR_4_6
	  hw_version: sx1262
	  rf_switch: true
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
