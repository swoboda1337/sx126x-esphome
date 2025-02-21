#pragma once

#include "sx126x_reg.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <vector>

namespace esphome {
namespace sx126x {

enum SX126xBw : uint8_t {
  SX126X_BW_2_6,
  SX126X_BW_3_1,
  SX126X_BW_3_9,
  SX126X_BW_5_2,
  SX126X_BW_6_3,
  SX126X_BW_7_8,
  SX126X_BW_10_4,
  SX126X_BW_12_5,
  SX126X_BW_15_6,
  SX126X_BW_20_8,
  SX126X_BW_25_0,
  SX126X_BW_31_3,
  SX126X_BW_41_7,
  SX126X_BW_50_0,
  SX126X_BW_62_5,
  SX126X_BW_83_3,
  SX126X_BW_100_0,
  SX126X_BW_125_0,
  SX126X_BW_166_7,
  SX126X_BW_200_0,
  SX126X_BW_250_0,
  SX126X_BW_500_0,
};

class SX126x : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_8MHZ> {
 public:
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void set_bandwidth(SX126xBw bandwidth) { this->bandwidth_ = bandwidth; }
  void set_bitrate(uint32_t bitrate) { this->bitrate_ = bitrate; }
  void set_bitsync(bool bitsync) { this->bitsync_ = bitsync; }
  void set_busy_pin(InternalGPIOPin *busy_pin) { this->busy_pin_ = busy_pin; }
  void set_coding_rate(uint8_t coding_rate) { this->coding_rate_ = coding_rate; }
  void set_crc_enable(bool crc_enable) { this->crc_enable_ = crc_enable; }
  void set_deviation(uint32_t deviation) { this->deviation_ = deviation; }
  void set_dio1_pin(InternalGPIOPin *dio1_pin) { this->dio1_pin_ = dio1_pin; }
  void set_frequency(uint32_t frequency) { this->frequency_ = frequency; }
  void set_mode_rx();
  void set_mode_standby();
  void set_mode_tx();
  void set_modulation(uint8_t modulation) { this->modulation_ = modulation; }
  void set_pa_pin(uint8_t pin) { this->pa_pin_ = pin; }
  void set_pa_power(uint8_t power) { this->pa_power_ = power; }
  void set_pa_ramp(uint8_t ramp) { this->pa_ramp_ = ramp; }
  void set_payload_length(uint8_t payload_length) { this->payload_length_ = payload_length; }
  void set_preamble_errors(uint8_t preamble_errors) { this->preamble_errors_ = preamble_errors; }
  void set_preamble_polarity(uint8_t preamble_polarity) { this->preamble_polarity_ = preamble_polarity; }
  void set_preamble_size(uint16_t preamble_size) { this->preamble_size_ = preamble_size; }
  void set_rst_pin(InternalGPIOPin *rst_pin) { this->rst_pin_ = rst_pin; }
  void set_rx_floor(float floor) { this->rx_floor_ = floor; }
  void set_rx_start(bool start) { this->rx_start_ = start; }
  void set_shaping(uint8_t shaping) { this->shaping_ = shaping; }
  void set_spreading_factor(uint8_t spreading_factor) { this->spreading_factor_ = spreading_factor; }
  void set_sync_value(const std::vector<uint8_t> &sync_value) { this->sync_value_ = sync_value; }
  void run_image_cal();
  void configure();
  void transmit_packet(const std::vector<uint8_t> &packet);
  Trigger<std::vector<uint8_t>, float, float> *get_packet_trigger() const { return this->packet_trigger_; };

 protected:
  void configure_fsk_ook_();
  void configure_lora_();
  void set_mode_(SX126xOpMode mode);
  uint8_t read_fifo_(uint8_t offset, std::vector<uint8_t> &packet);
  void write_fifo_(uint8_t opcode, std::vector<uint8_t> &packet);
  void write_opcode_(uint8_t opcode, uint8_t *data, uint8_t size);
  uint8_t read_opcode_(uint8_t opcode, uint8_t *data, uint8_t size);
  void write_register_(uint16_t reg, uint8_t *data, uint8_t size);
  void write_register_(uint16_t reg, uint8_t data);
  void read_register_(uint16_t reg, uint8_t *data, uint8_t size);
  uint8_t read_register_(uint16_t reg);
  void wait_busy_();
  uint8_t wakeup_();
  Trigger<std::vector<uint8_t>, float, float> *packet_trigger_{new Trigger<std::vector<uint8_t>, float, float>()};
  std::vector<uint8_t> sync_value_;
  InternalGPIOPin *busy_pin_{nullptr};
  InternalGPIOPin *dio1_pin_{nullptr};
  InternalGPIOPin *rst_pin_{nullptr};
  SX126xBw bandwidth_;
  char version_[16];
  uint32_t bitrate_;
  uint32_t deviation_;
  uint32_t frequency_;
  uint32_t payload_length_;
  uint16_t preamble_size_;
  uint8_t coding_rate_;
  uint8_t modulation_;
  uint8_t pa_pin_;
  uint8_t pa_power_;
  uint8_t pa_ramp_;
  uint8_t preamble_errors_;
  uint8_t preamble_polarity_;
  uint8_t shaping_;
  uint8_t spreading_factor_;
  float rx_floor_;
  bool bitsync_;
  bool crc_enable_;
  bool rx_start_;
};

}  // namespace sx126x
}  // namespace esphome
