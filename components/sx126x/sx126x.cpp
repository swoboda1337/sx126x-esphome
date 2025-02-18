#include "sx126x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cinttypes>

namespace esphome {
namespace sx126x {

static const char *const TAG = "sx126x";
static const uint32_t FXOSC = 32000000u;
static const uint16_t RAMP[16] = {3400, 2000, 1000, 500, 250, 125, 100, 62, 50, 40, 31, 25, 20, 15, 12, 10};
static const uint32_t BW_HZ[22] = {2604,  3125,  3906,  5208,  6250,  7812,   10416,  12500,  15625,  20833,  25000,
                                   31250, 41666, 50000, 62500, 83333, 100000, 125000, 166666, 200000, 250000, 500000};
static const uint8_t BW_LORA[22] = {LORA_BW_7,   LORA_BW_7,   LORA_BW_7,   LORA_BW_7,   LORA_BW_7,   LORA_BW_7,  LORA_BW_10, LORA_BW_15,
                                    LORA_BW_15,  LORA_BW_20,  LORA_BW_31,  LORA_BW_31,  LORA_BW_41,  LORA_BW_62, LORA_BW_62, LORA_BW_125,
                                    LORA_BW_125, LORA_BW_125, LORA_BW_250, LORA_BW_250, LORA_BW_250, LORA_BW_500};
static const uint8_t BW_FSK_OOK[22] = {RX_BW_2_6,   RX_BW_3_1,   RX_BW_3_9,   RX_BW_5_2,  RX_BW_6_3,   RX_BW_7_8,
                                       RX_BW_10_4,  RX_BW_12_5,  RX_BW_15_6,  RX_BW_20_8, RX_BW_25_0,  RX_BW_31_3,
                                       RX_BW_41_7,  RX_BW_50_0,  RX_BW_62_5,  RX_BW_83_3, RX_BW_100_0, RX_BW_125_0,
                                       RX_BW_166_7, RX_BW_200_0, RX_BW_250_0, RX_BW_250_0};

void SX126x::read_opcode_(uint8_t opcode, uint8_t *data, uint8_t size) {
  this->enable();
  this->wait_busy_();
  this->write_byte(opcode);
  this->write_byte(0x00);
  this->read_array(data, size);
  this->disable();
}

void SX126x::write_opcode_(uint8_t opcode, uint8_t *data, uint8_t size) {
  this->enable();
  this->wait_busy_();
  this->write_byte(opcode);
  this->write_array(data, size);
  this->disable();
}

void SX126x::read_register_(uint16_t reg, uint8_t *data, uint8_t size) {
  this->enable();
  this->wait_busy_();
  this->write_byte(RADIO_READ_REGISTER);
  this->write_byte((reg >> 8) & 0xFF);
  this->write_byte((reg >> 0) & 0xFF);
  this->write_byte(0x00);
  this->read_array(data, size);
  this->disable();
}

void SX126x::write_register_(uint16_t reg, uint8_t *data, uint8_t size) {
  this->enable();
  this->wait_busy_();
  this->write_byte(RADIO_WRITE_REGISTER);
  this->write_byte((reg >> 8) & 0xFF);
  this->write_byte((reg >> 0) & 0xFF);
  this->write_array(data, size);
  this->disable();
}

uint8_t SX126x::read_register_(uint16_t reg) {
  uint8_t data;
  this->read_register_(reg, &data, 1);
  return data;
}

void SX126x::write_register_(uint16_t reg, uint8_t data) {
  this->write_register_(reg, &data, 1);
}

void SX126x::read_fifo_(std::vector<uint8_t> &packet) {
  this->enable();
  this->write_byte(REG_FIFO & 0x7F);
  this->read_array(packet.data(), packet.size());
  this->disable();
}

void SX126x::write_fifo_(const std::vector<uint8_t> &packet) {
  this->enable();
  this->write_byte(REG_FIFO | 0x80);
  this->write_array(packet.data(), packet.size());
  this->disable();
}

void SX126x::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX126x...");

  // setup busy and reset
  this->busy_pin_->setup();
  this->rst_pin_->setup();

  // setup dio1
  if (this->dio1_pin_) {
    this->dio1_pin_->setup();
  }

  // start spi
  this->spi_setup();

  // configure rf
  this->configure();
}

#define RADIOLIB_SX126X_REG_VERSION_STRING                      0x0320

void SX126x::configure() {
  uint8_t buf[6];

  // toggle chip reset
  this->rst_pin_->digital_write(false);
  delayMicroseconds(2000);
  this->rst_pin_->digital_write(true);
  delayMicroseconds(6000);
  this->wait_busy_();

  // check silicon version to make sure hw is ok
  this->read_register_(0x0320, (uint8_t*) this->version_, 16);
  if (strncmp(this->version_, "SX126", 5) != 0) {
    this->mark_failed();
    return;
  }

  // setup buffer
  buf[0] = 0;
  buf[1] = 0;
  this->write_opcode_(RADIO_SET_BUFFERBASEADDRESS, buf, 2);

  // setup packet type
  buf[0] = this->modulation_;
  this->write_opcode_(RADIO_SET_PACKETTYPE, buf, 1);

  // this->read_opcode_(RADIO_GET_STATUS, buf, 1);
  // ESP_LOGE(TAG, "%02x", buf[0]);
  // // enter sleep mode
  // this->write_register_(REG_OP_MODE, MODE_SLEEP);
  // delayMicroseconds(1000);

  // // set freq
  // uint64_t frf = ((uint64_t) this->frequency_ << 19) / FXOSC;
  // this->write_register_(REG_FRF_MSB, (uint8_t) ((frf >> 16) & 0xFF));
  // this->write_register_(REG_FRF_MID, (uint8_t) ((frf >> 8) & 0xFF));
  // this->write_register_(REG_FRF_LSB, (uint8_t) ((frf >> 0) & 0xFF));

  uint32_t freq = (uint32_t) std::ceil((float) this->frequency_ / FREQ_STEP);
  buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
  buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
  buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
  buf[3] = (uint8_t) (freq & 0xFF);
  this->write_opcode_(RADIO_SET_RFFREQUENCY, buf, 4);

  // // enter standby mode
  // this->write_register_(REG_OP_MODE, MODE_STDBY);
  // delayMicroseconds(1000);

  // // run image cal
  // this->run_image_cal();

  // // set correct modulation and go back to sleep
  // this->write_register_(REG_OP_MODE, this->modulation_ | MODE_SLEEP);
  // delayMicroseconds(1000);

  // // config pa
  // if (this->pa_pin_ == PA_PIN_BOOST) {
  //   this->pa_power_ = std::max(this->pa_power_, (uint8_t) 2);
  //   this->pa_power_ = std::min(this->pa_power_, (uint8_t) 17);
  //   this->write_register_(REG_PA_CONFIG, (this->pa_power_ - 2) | this->pa_pin_ | PA_MAX_POWER);
  // } else {
  //   this->pa_power_ = std::min(this->pa_power_, (uint8_t) 14);
  //   this->write_register_(REG_PA_CONFIG, (this->pa_power_ - 0) | this->pa_pin_ | PA_MAX_POWER);
  // }
  // if (this->modulation_ != MOD_LORA) {
  //   this->write_register_(REG_PA_RAMP, this->pa_ramp_ | this->shaping_);
  // } else {
  //   this->write_register_(REG_PA_RAMP, this->pa_ramp_);
  // }

  // configure modem
  if (this->modulation_ == MOD_LORA) {
    // set modulation params
    float duration = 1000.0f * std::pow(2, this->spreading_factor_) / BW_HZ[this->bandwidth_];
    buf[0] = this->spreading_factor_;
    buf[1] = BW_LORA[this->bandwidth_];
    buf[2] = this->coding_rate_;
    buf[3] = (duration > 16) ? 0x01 : 0x00;
    this->write_opcode_(RADIO_SET_MODULATIONPARAMS, buf, 4);

    // set packet params
    buf[0] = (this->preamble_size_ >> 8) & 0xFF;
    buf[1] = (this->preamble_size_ >> 0) & 0xFF;
    buf[2] = (this->payload_length_ > 0) ? 0x01 : 0x00;
    buf[3] = this->payload_length_;
    buf[4] = (this->crc_enable_) ? 0x01 : 0x00;
    buf[5] = 0x00;
    this->write_opcode_(RADIO_SET_PACKETPARAMS, buf, 6);
  } else {
    this->configure_fsk_ook_();
  }

  // // switch to rx or standby
  // if (this->rx_start_) {
  //   this->set_mode_rx();
  // } else {
  //   this->set_mode_standby();
  // }
}

void SX126x::configure_fsk_ook_() {
  // set the channel bw
  this->write_register_(REG_RX_BW, BW_FSK_OOK[this->bandwidth_]);

  // set fdev
  uint32_t fdev = std::min((this->deviation_ * 4096) / 250000, (uint32_t) 0x3FFF);
  this->write_register_(REG_FDEV_MSB, (uint8_t) ((fdev >> 8) & 0xFF));
  this->write_register_(REG_FDEV_LSB, (uint8_t) ((fdev >> 0) & 0xFF));

  // set bitrate
  uint64_t bitrate = (FXOSC + this->bitrate_ / 2) / this->bitrate_;  // round up
  this->write_register_(REG_BITRATE_MSB, (uint8_t) ((bitrate >> 8) & 0xFF));
  this->write_register_(REG_BITRATE_LSB, (uint8_t) ((bitrate >> 0) & 0xFF));

  // configure rx and afc
  uint8_t trigger = (this->preamble_size_ > 0) ? TRIGGER_PREAMBLE : TRIGGER_RSSI;
  this->write_register_(REG_AFC_FEI, AFC_AUTO_CLEAR_ON);
  if (this->modulation_ == MOD_FSK) {
    this->write_register_(REG_RX_CONFIG, AFC_AUTO_ON | AGC_AUTO_ON | trigger);
  } else {
    this->write_register_(REG_RX_CONFIG, AGC_AUTO_ON | trigger);
  }

  // configure packet mode
  if (this->payload_length_ > 0) {
    uint8_t crc_mode = (this->crc_enable_) ? CRC_ON : CRC_OFF;
    this->write_register_(REG_FIFO_THRESH, TX_START_FIFO_LEVEL | (this->payload_length_ - 1));
    this->write_register_(REG_PACKET_CONFIG_1, crc_mode);
    this->write_register_(REG_PACKET_CONFIG_2, PACKET_MODE);
    this->write_register_(REG_PAYLOAD_LENGTH_LSB, this->payload_length_);
  } else {
    this->write_register_(REG_PACKET_CONFIG_2, CONTINUOUS_MODE);
  }
  this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_00);

  // config bit synchronizer
  uint8_t polarity = (this->preamble_polarity_ == 0xAA) ? PREAMBLE_AA : PREAMBLE_55;
  if (!this->sync_value_.empty()) {
    uint8_t size = this->sync_value_.size() - 1;
    this->write_register_(REG_SYNC_CONFIG, AUTO_RESTART_PLL_LOCK | polarity | SYNC_ON | size);
    for (uint32_t i = 0; i < this->sync_value_.size(); i++) {
      this->write_register_(REG_SYNC_VALUE1 + i, this->sync_value_[i]);
    }
  } else {
    this->write_register_(REG_SYNC_CONFIG, AUTO_RESTART_PLL_LOCK | polarity);
  }

  // config preamble detector
  if (this->preamble_size_ > 0) {
    uint8_t size = (std::min(this->preamble_size_, (uint16_t) 3) - 1) << PREAMBLE_BYTES_SHIFT;
    uint8_t errors = this->preamble_errors_;
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_ON | size | errors);
  } else {
    this->write_register_(REG_PREAMBLE_DETECT, PREAMBLE_DETECTOR_OFF);
  }
  this->write_register_(REG_PREAMBLE_SIZE_MSB, this->preamble_size_ >> 16);
  this->write_register_(REG_PREAMBLE_SIZE_LSB, this->preamble_size_ & 0xFF);

  // config sync generation and setup ook threshold
  uint8_t bitsync = this->bitsync_ ? BIT_SYNC_ON : BIT_SYNC_OFF;
  this->write_register_(REG_OOK_PEAK, bitsync | OOK_THRESH_STEP_0_5 | OOK_THRESH_PEAK);
  this->write_register_(REG_OOK_AVG, OOK_AVG_RESERVED | OOK_THRESH_DEC_1_8);

  // set rx floor
  this->write_register_(REG_OOK_FIX, 256 + int(this->rx_floor_ * 2.0));
  this->write_register_(REG_RSSI_THRESH, std::abs(int(this->rx_floor_ * 2.0)));
}

void SX126x::configure_lora_() {
  // config modem
  // uint8_t header_mode = this->payload_length_ > 0 ? IMPLICIT_HEADER : EXPLICIT_HEADER;
  // uint8_t crc_mode = (this->crc_enable_) ? RX_PAYLOAD_CRC_ON : RX_PAYLOAD_CRC_OFF;
  // uint8_t spreading_factor = this->spreading_factor_ << SPREADING_FACTOR_SHIFT;
  // this->write_register_(REG_MODEM_CONFIG1, BW_LORA[this->bandwidth_] | this->coding_rate_ | header_mode);
  // this->write_register_(REG_MODEM_CONFIG2, spreading_factor | crc_mode);

  // // config fifo and payload length
  // this->write_register_(REG_FIFO_TX_BASE_ADDR, 0x00);
  // this->write_register_(REG_FIFO_RX_BASE_ADDR, 0x00);
  // this->write_register_(REG_PAYLOAD_LENGTH, std::max(this->payload_length_, (uint32_t) 1));

  // // config preamble
  // if (this->preamble_size_ >= 6) {
  //   this->write_register_(REG_PREAMBLE_LEN_MSB, this->preamble_size_ >> 16);
  //   this->write_register_(REG_PREAMBLE_LEN_LSB, this->preamble_size_ & 0xFF);
  // }

  // // optimize detection
  // float duration = 1000.0f * std::pow(2, this->spreading_factor_) / BW_HZ[this->bandwidth_];
  // if (duration > 16) {
  //   this->write_register_(REG_MODEM_CONFIG3, MODEM_AGC_AUTO_ON | LOW_DATA_RATE_OPTIMIZE_ON);
  // } else {
  //   this->write_register_(REG_MODEM_CONFIG3, MODEM_AGC_AUTO_ON);
  // }
  // if (this->spreading_factor_ == 6) {
  //   this->write_register_(REG_DETECT_OPTIMIZE, 0xC5);
  //   this->write_register_(REG_DETECT_THRESHOLD, 0x0C);
  // } else {
  //   this->write_register_(REG_DETECT_OPTIMIZE, 0xC3);
  //   this->write_register_(REG_DETECT_THRESHOLD, 0x0A);
  // }

  // // config sync word
  // if (!this->sync_value_.empty()) {
  //   this->write_register_(REG_SYNC_WORD, this->sync_value_[0]);
  // }
}

void SX126x::transmit_packet(const std::vector<uint8_t> &packet) {
  // if (this->payload_length_ > 0 && this->payload_length_ != packet.size()) {
  //   ESP_LOGE(TAG, "Packet size does not match payload length");
  //   return;
  // }
  // if (packet.empty() || packet.size() > 256) {
  //   ESP_LOGE(TAG, "Packet size out of range");
  //   return;
  // }
  // if (this->modulation_ == MOD_LORA) {
  //   this->set_mode_standby();
  //   if (this->payload_length_ == 0) {
  //     this->write_register_(REG_PAYLOAD_LENGTH, packet.size());
  //   }
  //   this->write_register_(REG_IRQ_FLAGS, 0xFF);
  //   this->write_register_(REG_FIFO_ADDR_PTR, 0);
  //   this->write_fifo_(packet);
  //   this->set_mode_tx();
  // } else {
  //   this->set_mode_tx();
  //   this->write_fifo_(packet);
  // }
  // uint32_t start = millis();
  // while (!this->dio1_pin_->digital_read()) {
  //   if (millis() - start > 4000) {
  //     ESP_LOGE(TAG, "Transmit packet failure");
  //     break;
  //   }
  // }
  // if (this->rx_start_) {
  //   this->set_mode_rx();
  // } else {
  //   this->set_mode_standby();
  // }
}

void SX126x::loop() {
  // if (this->modulation_ == MOD_LORA) {
  //   if (this->dio1_pin_->digital_read()) {
  //     if ((this->read_register_(REG_IRQ_FLAGS) & PAYLOAD_CRC_ERROR) == 0) {
  //       uint8_t bytes = this->read_register_(REG_NB_RX_BYTES);
  //       uint8_t addr = this->read_register_(REG_FIFO_RX_CURR_ADDR);
  //       uint8_t rssi = this->read_register_(REG_PKT_RSSI_VALUE);
  //       uint8_t snr = this->read_register_(REG_PKT_SNR_VALUE);
  //       std::vector<uint8_t> packet(bytes);
  //       this->write_register_(REG_FIFO_ADDR_PTR, addr);
  //       this->read_fifo_(packet);
  //       if (this->frequency_ > 700000000) {
  //         this->packet_trigger_->trigger(packet, (float) rssi - 157, (float) snr / 4);
  //       } else {
  //         this->packet_trigger_->trigger(packet, (float) rssi - 164, (float) snr / 4);
  //       }
  //     }
  //     this->write_register_(REG_IRQ_FLAGS, 0xFF);
  //   }
  // } else if (this->payload_length_ > 0 && this->dio1_pin_->digital_read()) {
  //   std::vector<uint8_t> packet(this->payload_length_);
  //   this->read_fifo_(packet);
  //   this->packet_trigger_->trigger(packet, 0.0f, 0.0f);
  // }
}

void SX126x::run_image_cal() {
  // uint32_t start = millis();
  // this->write_register_(REG_IMAGE_CAL, AUTO_IMAGE_CAL_ON | IMAGE_CAL_START | TEMP_THRESHOLD_10C);
  // while (this->read_register_(REG_IMAGE_CAL) & IMAGE_CAL_RUNNING) {
  //   if (millis() - start > 20) {
  //     ESP_LOGE(TAG, "Image cal failure");
  //     break;
  //   }
  // }
}

void SX126x::set_mode_(SX126xOpMode mode) {
  // uint32_t start = millis();
  // this->write_register_(REG_OP_MODE, this->modulation_ | mode);
  // while (true) {
  //   uint8_t curr = this->read_register_(REG_OP_MODE) & MODE_MASK;
  //   if ((curr == mode) || (mode == MODE_RX && curr == MODE_RX_FS)) {
  //     break;
  //   }
  //   if (millis() - start > 20) {
  //     ESP_LOGE(TAG, "Set mode failure");
  //     break;
  //   }
  // }
}

void SX126x::set_mode_rx() {
  // this->set_mode_(MODE_RX);
  // if (this->modulation_ == MOD_LORA) {
  //   this->write_register_(REG_IRQ_FLAGS_MASK, 0x00);
  //   this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_00);
  // }
}

void SX126x::set_mode_tx() {
  // this->set_mode_(MODE_TX);
  // if (this->modulation_ == MOD_LORA) {
  //   this->write_register_(REG_IRQ_FLAGS_MASK, 0x00);
  //   this->write_register_(REG_DIO_MAPPING1, DIO0_MAPPING_01);
  // }
}

void SX126x::set_mode_standby() { } //this->set_mode_(MODE_STDBY); }

void SX126x::wait_busy_() {
  uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > 1000) {
      ESP_LOGE(TAG, "Wait busy timout");
      break;
    }
  }
}

void SX126x::dump_config() {
  ESP_LOGCONFIG(TAG, "SX126x:");
  ESP_LOGCONFIG(TAG, "  HW Version: %15s", this->version_);
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  BUSY Pin: ", this->busy_pin_);
  LOG_PIN("  RST Pin: ", this->rst_pin_);
  LOG_PIN("  DIO0 Pin: ", this->dio1_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %" PRIu32 " Hz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Bandwidth: %" PRIu32 " Hz", BW_HZ[this->bandwidth_]);
  ESP_LOGCONFIG(TAG, "  PA Pin: %s", this->pa_pin_ == PA_PIN_BOOST ? "BOOST" : "RFO");
  ESP_LOGCONFIG(TAG, "  PA Power: %" PRIu32 " dBm", this->pa_power_);
  ESP_LOGCONFIG(TAG, "  PA Ramp: %" PRIu16 " us", RAMP[this->pa_ramp_]);
  if (this->shaping_ == CUTOFF_BR_X_2) {
    ESP_LOGCONFIG(TAG, "  Shaping: CUTOFF_BR_X_2");
  } else if (this->shaping_ == CUTOFF_BR_X_1) {
    ESP_LOGCONFIG(TAG, "  Shaping: CUTOFF_BR_X_1");
  } else if (this->shaping_ == GAUSSIAN_BT_0_3) {
    ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_0_3");
  } else if (this->shaping_ == GAUSSIAN_BT_0_5) {
    ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_0_5");
  } else if (this->shaping_ == GAUSSIAN_BT_1_0) {
    ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_1_0");
  } else {
    ESP_LOGCONFIG(TAG, "  Shaping: NONE");
  }
  if (this->modulation_ == MOD_FSK) {
    ESP_LOGCONFIG(TAG, "  Deviation: %" PRIu32 " Hz", this->deviation_);
  }
  if (this->modulation_ == PACKET_TYPE_LORA) {
    ESP_LOGCONFIG(TAG, "  Modulation: %s", "LORA");
    ESP_LOGCONFIG(TAG, "  Spreading Factor: %" PRIu8, this->spreading_factor_);
    if (this->coding_rate_ == CODING_RATE_4_5) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/5");
    } else if (this->coding_rate_ == CODING_RATE_4_6) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/6");
    } else if (this->coding_rate_ == CODING_RATE_4_7) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/7");
    } else {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/8");
    }
    if (!this->sync_value_.empty()) {
      ESP_LOGCONFIG(TAG, "  Sync Value: 0x%02x", this->sync_value_[0]);
    }
    if (this->preamble_size_ > 0) {
      ESP_LOGCONFIG(TAG, "  Preamble Size: %" PRIu16, this->preamble_size_);
    }
  } else {
    ESP_LOGCONFIG(TAG, "  Modulation: %s", this->modulation_ == MOD_FSK ? "FSK" : "OOK");
    ESP_LOGCONFIG(TAG, "  Bitrate: %" PRIu32 "b/s", this->bitrate_);
    ESP_LOGCONFIG(TAG, "  Bitsync: %s", TRUEFALSE(this->bitsync_));
    ESP_LOGCONFIG(TAG, "  Rx Start: %s", TRUEFALSE(this->rx_start_));
    ESP_LOGCONFIG(TAG, "  Rx Floor: %.1f dBm", this->rx_floor_);
    if (this->preamble_size_ > 0) {
      ESP_LOGCONFIG(TAG, "  Preamble Size: %" PRIu16, this->preamble_size_);
      ESP_LOGCONFIG(TAG, "  Preamble Polarity: 0x%X", this->preamble_polarity_);
      ESP_LOGCONFIG(TAG, "  Preamble Errors: %" PRIu8, this->preamble_errors_);
    }
    if (!this->sync_value_.empty()) {
      ESP_LOGCONFIG(TAG, "  Sync Value: 0x%s", format_hex(this->sync_value_).c_str());
    }
  }
  if (this->payload_length_ > 0) {
    ESP_LOGCONFIG(TAG, "  Payload Length: %" PRIu32, this->payload_length_);
  }
  if (this->payload_length_ > 0 || this->modulation_ == MOD_LORA) {
    ESP_LOGCONFIG(TAG, "  CRC Enable: %s", TRUEFALSE(this->crc_enable_));
  }
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Configuring SX126x failed");
  }
}

}  // namespace sx126x
}  // namespace esphome
