#include "sx126x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace sx126x {

static const char *const TAG = "sx126x";
static const uint16_t RAMP[8] = {10, 20, 40, 80, 200, 800, 1700, 3400};
static const uint32_t BW_HZ[31] = {4800,  5800,  7300,  9700,   11700,  14600,  19500,  23400,  29300,  39000,  46900,
                                   58600, 78200, 93800, 117300, 156200, 187200, 234300, 312000, 373600, 467000, 7810,
                                   10420, 15630, 20830, 31250,  41670,  62500,  125000, 250000, 500000};
static const uint8_t BW_LORA[10] = {LORA_BW_7810,  LORA_BW_10420, LORA_BW_15630,  LORA_BW_20830,  LORA_BW_31250,
                                    LORA_BW_41670, LORA_BW_62500, LORA_BW_125000, LORA_BW_250000, LORA_BW_500000};
static const uint8_t BW_FSK[21] = {
    FSK_BW_4800,   FSK_BW_5800,   FSK_BW_7300,   FSK_BW_9700,   FSK_BW_11700,  FSK_BW_14600,  FSK_BW_19500,
    FSK_BW_23400,  FSK_BW_29300,  FSK_BW_39000,  FSK_BW_46900,  FSK_BW_58600,  FSK_BW_78200,  FSK_BW_93800,
    FSK_BW_117300, FSK_BW_156200, FSK_BW_187200, FSK_BW_234300, FSK_BW_312000, FSK_BW_373600, FSK_BW_467000};

uint8_t SX126x::wakeup_() {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(RADIO_GET_STATUS);
  uint8_t status = this->transfer_byte(0x00);
  this->disable();
  this->wait_busy_();
  return status;
}

uint8_t SX126x::read_fifo_(uint8_t offset, std::vector<uint8_t> &packet) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(RADIO_READ_BUFFER);
  this->transfer_byte(offset);
  uint8_t status = this->transfer_byte(0x00);
  for (uint8_t &byte : packet) {
    byte = this->transfer_byte(0x00);
  }
  this->disable();
  this->wait_busy_();
  return status;
}

void SX126x::write_fifo_(uint8_t offset, const std::vector<uint8_t> &packet) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(RADIO_WRITE_BUFFER);
  this->transfer_byte(offset);
  for (const uint8_t &byte : packet) {
    this->transfer_byte(byte);
  }
  this->disable();
  this->wait_busy_();
}

uint8_t SX126x::read_opcode_(uint8_t opcode, uint8_t *data, uint8_t size) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(opcode);
  uint8_t status = this->transfer_byte(0x00);
  for (int32_t i = 0; i < size; i++) {
    data[i] = this->transfer_byte(0x00);
  }
  this->disable();
  this->wait_busy_();
  return status;
}

void SX126x::write_opcode_(uint8_t opcode, uint8_t *data, uint8_t size) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(opcode);
  for (int32_t i = 0; i < size; i++) {
    this->transfer_byte(data[i]);
  }
  this->disable();
  this->wait_busy_();
}

void SX126x::read_register_(uint16_t reg, uint8_t *data, uint8_t size) {
  this->wait_busy_();
  this->enable();
  this->write_byte(RADIO_READ_REGISTER);
  this->write_byte((reg >> 8) & 0xFF);
  this->write_byte((reg >> 0) & 0xFF);
  this->write_byte(0x00);
  for (int32_t i = 0; i < size; i++) {
    data[i] = this->transfer_byte(0x00);
  }
  this->disable();
  this->wait_busy_();
}

void SX126x::write_register_(uint16_t reg, uint8_t *data, uint8_t size) {
  this->wait_busy_();
  this->enable();
  this->write_byte(RADIO_WRITE_REGISTER);
  this->write_byte((reg >> 8) & 0xFF);
  this->write_byte((reg >> 0) & 0xFF);
  for (int32_t i = 0; i < size; i++) {
    this->transfer_byte(data[i]);
  }
  this->disable();
  this->wait_busy_();
}

void SX126x::setup() {
  ESP_LOGCONFIG(TAG, "Running setup");

  // setup pins
  this->busy_pin_->setup();
  this->rst_pin_->setup();
  this->dio1_pin_->setup();

  // start spi
  this->spi_setup();

  // configure rf
  this->configure();
}

void SX126x::configure() {
  uint8_t buf[8];

  // toggle chip reset
  this->rst_pin_->digital_write(true);
  delayMicroseconds(5000);
  this->rst_pin_->digital_write(false);
  delayMicroseconds(2000);
  this->rst_pin_->digital_write(true);
  delayMicroseconds(5000);

  // wakeup
  this->wakeup_();

  // config tcxo
  if (this->tcxo_voltage_ != TCXO_CTRL_NONE) {
    uint32_t delay = this->tcxo_delay_ >> 6;
    buf[0] = this->tcxo_voltage_;
    buf[1] = (delay >> 16) & 0xFF;
    buf[2] = (delay >> 8) & 0xFF;
    buf[3] = (delay >> 0) & 0xFF;
    this->write_opcode_(RADIO_SET_TCXOMODE, buf, 4);
    buf[0] = 0x7F;
    this->write_opcode_(RADIO_CALIBRATE, buf, 1);
  }

  // clear errors
  buf[0] = 0x00;
  buf[1] = 0x00;
  this->write_opcode_(RADIO_CLR_ERROR, buf, 2);

  // rf switch
  if (this->rf_switch_) {
    buf[0] = 0x01;
    this->write_opcode_(RADIO_SET_RFSWITCHMODE, buf, 1);
  }

  // check silicon version to make sure hw is ok
  this->read_register_(REG_VERSION_STRING, (uint8_t *) this->version_, 16);
  if (strncmp(this->version_, "SX126", 5) != 0 && strncmp(this->version_, "LLCC68", 6) != 0) {
    this->mark_failed();
    return;
  }

  // setup packet type
  buf[0] = this->modulation_;
  this->write_opcode_(RADIO_SET_PACKETTYPE, buf, 1);

  // calibrate image
  this->run_image_cal();

  // set frequency
  uint64_t freq = ((uint64_t) this->frequency_ << 25) / XTAL_FREQ;
  buf[0] = (uint8_t) ((freq >> 24) & 0xFF);
  buf[1] = (uint8_t) ((freq >> 16) & 0xFF);
  buf[2] = (uint8_t) ((freq >> 8) & 0xFF);
  buf[3] = (uint8_t) (freq & 0xFF);
  this->write_opcode_(RADIO_SET_RFFREQUENCY, buf, 4);

  // configure pa
  int8_t pa_power = this->pa_power_;
  if (this->hw_version_ == "sx1261") {
    if (pa_power == 15) {
      uint8_t cfg[4] = {0x06, 0x00, 0x01, 0x01};
      this->write_opcode_(RADIO_SET_PACONFIG, cfg, 4);
    } else {
      uint8_t cfg[4] = {0x04, 0x00, 0x01, 0x01};
      this->write_opcode_(RADIO_SET_PACONFIG, cfg, 4);
    }
    pa_power = std::max(pa_power, (int8_t) -3);
    pa_power = std::min(pa_power, (int8_t) 14);
    buf[0] = 0x18;  // max 80 mA
    this->write_register_(REG_OCP, buf, 1);
  } else {
    uint8_t cfg[4] = {0x04, 0x07, 0x00, 0x01};
    this->write_opcode_(RADIO_SET_PACONFIG, cfg, 4);
    pa_power = std::max(pa_power, (int8_t) -3);
    pa_power = std::min(pa_power, (int8_t) 22);
    buf[0] = 0x38;  // max 140 mA
    this->write_register_(REG_OCP, buf, 1);
  }
  buf[0] = pa_power;
  buf[1] = this->pa_ramp_;
  this->write_opcode_(RADIO_SET_TXPARAMS, buf, 2);

  // configure modem
  if (this->modulation_ == PACKET_TYPE_LORA) {
    // set modulation params
    float duration = 1000.0f * std::pow(2, this->spreading_factor_) / BW_HZ[this->bandwidth_];
    buf[0] = this->spreading_factor_;
    buf[1] = BW_LORA[this->bandwidth_ - SX126X_BW_7810];
    buf[2] = this->coding_rate_;
    buf[3] = (duration > 16.38f) ? 0x01 : 0x00;
    this->write_opcode_(RADIO_SET_MODULATIONPARAMS, buf, 4);

    // set packet params and sync word
    this->set_packet_params_(this->payload_length_);
    if (this->sync_value_.size() == 2) {
      for (uint32_t i = 0; i < this->sync_value_.size(); i++) {
        uint8_t data = this->sync_value_[i];
        this->write_register_(REG_LORA_SYNCWORD + i, &data, 1);
      }
    }
  } else {
    // set modulation params
    uint32_t bitrate = ((uint64_t) XTAL_FREQ * 32) / this->bitrate_;
    uint32_t fdev = ((uint64_t) this->deviation_ << 25) / XTAL_FREQ;
    buf[0] = (bitrate >> 16) & 0xFF;
    buf[1] = (bitrate >> 8) & 0xFF;
    buf[2] = (bitrate >> 0) & 0xFF;
    buf[3] = this->shaping_;
    buf[4] = BW_FSK[this->bandwidth_ - SX126X_BW_4800];
    buf[5] = (fdev >> 16) & 0xFF;
    buf[6] = (fdev >> 8) & 0xFF;
    buf[7] = (fdev >> 0) & 0xFF;
    this->write_opcode_(RADIO_SET_MODULATIONPARAMS, buf, 8);

    // set packet params and sync word
    this->set_packet_params_(this->payload_length_);
    if (!this->sync_value_.empty()) {
      for (uint32_t i = 0; i < this->sync_value_.size(); i++) {
        uint8_t data = this->sync_value_[i];
        this->write_register_(REG_GFSK_SYNCWORD + i, &data, 1);
      }
    }
  }

  // switch to rx or sleep
  if (this->rx_start_) {
    this->set_mode_rx();
  } else {
    this->set_mode_sleep();
  }
}

size_t SX126x::get_max_packet_size() {
  if (this->payload_length_ > 0) {
    return this->payload_length_;
  }
  return 256;
}

void SX126x::set_packet_params_(uint8_t payload_length) {
  uint8_t buf[9];
  if (this->modulation_ == PACKET_TYPE_LORA) {
    buf[0] = (this->preamble_size_ >> 8) & 0xFF;
    buf[1] = (this->preamble_size_ >> 0) & 0xFF;
    buf[2] = (this->payload_length_ > 0) ? 0x01 : 0x00;
    buf[3] = payload_length;
    buf[4] = (this->crc_enable_) ? 0x01 : 0x00;
    buf[5] = 0x00;
    this->write_opcode_(RADIO_SET_PACKETPARAMS, buf, 6);
  } else {
    uint16_t preamble_size = this->preamble_size_ * 8;
    buf[0] = (preamble_size >> 8) & 0xFF;
    buf[1] = (preamble_size >> 0) & 0xFF;
    buf[2] = (this->preamble_detect_ > 0) ? ((this->preamble_detect_ - 1) | 0x04) : 0x00;
    buf[3] = this->sync_value_.size() * 8;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = payload_length;
    buf[7] = this->crc_enable_ ? 0x06 : 0x01;
    buf[8] = 0x00;
    this->write_opcode_(RADIO_SET_PACKETPARAMS, buf, 9);
  }
}

void SX126x::transmit_packet(const std::vector<uint8_t> &packet) {
  if (this->payload_length_ > 0 && this->payload_length_ != packet.size()) {
    ESP_LOGE(TAG, "Packet size does not match config");
    return;
  }
  if (packet.empty() || packet.size() > this->get_max_packet_size()) {
    ESP_LOGE(TAG, "Packet size out of range");
    return;
  }
  this->set_mode_standby(STDBY_XOSC);
  if (this->payload_length_ == 0) {
    this->set_packet_params_(packet.size());
  }
  this->write_fifo_(0x00, packet);
  this->set_mode_tx();
  uint32_t start = millis();
  while (!this->dio1_pin_->digital_read()) {
    if (millis() - start > 4000) {
      ESP_LOGE(TAG, "Transmit packet failure");
      break;
    }
  }
  uint8_t buf[2];
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  this->write_opcode_(RADIO_CLR_IRQSTATUS, buf, 2);
  if (this->rx_start_) {
    this->set_mode_rx();
  } else {
    this->set_mode_sleep();
  }
}

void SX126x::call_listeners_(const std::vector<uint8_t> &packet, float rssi, float snr) {
  for (auto &listener : this->listeners_) {
    listener->on_packet(packet, rssi, snr);
  }
  this->packet_trigger_->trigger(packet, rssi, snr);
}

void SX126x::loop() {
  if (this->dio1_pin_->digital_read()) {
    uint16_t status;
    uint8_t buf[3];
    uint8_t rssi;
    int8_t snr;
    this->read_opcode_(RADIO_GET_IRQSTATUS, buf, 2);
    this->write_opcode_(RADIO_CLR_IRQSTATUS, buf, 2);
    status = (buf[0] << 8) | buf[1];
    if ((status & IRQ_RX_DONE) == IRQ_RX_DONE) {
      if ((status & IRQ_CRC_ERROR) != IRQ_CRC_ERROR) {
        this->read_opcode_(RADIO_GET_PACKETSTATUS, buf, 3);
        if (this->modulation_ == PACKET_TYPE_LORA) {
          rssi = buf[0];
          snr = buf[1];
        } else {
          rssi = buf[2];
          snr = 0;
        }
        this->read_opcode_(RADIO_GET_RXBUFFERSTATUS, buf, 2);
        std::vector<uint8_t> packet(buf[0]);
        this->read_fifo_(buf[1], packet);
        this->call_listeners_(packet, (float) rssi / -2.0f, (float) snr / 4.0f);
      }
    }
  }
}

void SX126x::run_image_cal() {
  uint8_t buf[2] = {0, 0};
  if (this->frequency_ > 900000000) {
    buf[0] = 0xE1;
    buf[1] = 0xE9;
  } else if (this->frequency_ > 850000000) {
    buf[0] = 0xD7;
    buf[1] = 0xD8;
  } else if (this->frequency_ > 770000000) {
    buf[0] = 0xC1;
    buf[1] = 0xC5;
  } else if (this->frequency_ > 460000000) {
    buf[0] = 0x75;
    buf[1] = 0x81;
  } else if (this->frequency_ > 425000000) {
    buf[0] = 0x6B;
    buf[1] = 0x6F;
  }
  if (buf[0] > 0 && buf[1] > 0) {
    this->write_opcode_(RADIO_CALIBRATEIMAGE, buf, 2);
  }
}

void SX126x::set_mode_rx() {
  uint8_t buf[8];

  // configure irq params
  uint16_t irq = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR;
  buf[0] = (irq >> 8) & 0xFF;
  buf[1] = (irq >> 0) & 0xFF;
  buf[2] = (irq >> 8) & 0xFF;
  buf[3] = (irq >> 0) & 0xFF;
  buf[4] = (IRQ_RADIO_NONE >> 8) & 0xFF;
  buf[5] = (IRQ_RADIO_NONE >> 0) & 0xFF;
  buf[6] = (IRQ_RADIO_NONE >> 8) & 0xFF;
  buf[7] = (IRQ_RADIO_NONE >> 0) & 0xFF;
  this->write_opcode_(RADIO_SET_DIOIRQPARAMS, buf, 8);

  // set timeout to 0
  buf[0] = 0x00;
  this->write_opcode_(RADIO_SET_LORASYMBTIMEOUT, buf, 1);

  // switch to continuous mode rx
  buf[0] = 0xFF;
  buf[1] = 0xFF;
  buf[2] = 0xFF;
  this->write_opcode_(RADIO_SET_RX, buf, 3);
}

void SX126x::set_mode_tx() {
  uint8_t buf[8];

  // configure irq params
  uint16_t irq = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
  buf[0] = (irq >> 8) & 0xFF;
  buf[1] = (irq >> 0) & 0xFF;
  buf[2] = (irq >> 8) & 0xFF;
  buf[3] = (irq >> 0) & 0xFF;
  buf[4] = (IRQ_RADIO_NONE >> 8) & 0xFF;
  buf[5] = (IRQ_RADIO_NONE >> 0) & 0xFF;
  buf[6] = (IRQ_RADIO_NONE >> 8) & 0xFF;
  buf[7] = (IRQ_RADIO_NONE >> 0) & 0xFF;
  this->write_opcode_(RADIO_SET_DIOIRQPARAMS, buf, 8);

  // switch to single mode tx
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  this->write_opcode_(RADIO_SET_TX, buf, 3);
}

void SX126x::set_mode_sleep() {
  uint8_t buf[1];
  buf[0] = 0x05;
  this->write_opcode_(RADIO_SET_SLEEP, buf, 1);
}

void SX126x::set_mode_standby(SX126xStandbyMode mode) {
  uint8_t buf[1];
  buf[0] = mode;
  this->write_opcode_(RADIO_SET_STANDBY, buf, 1);
}

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
  LOG_PIN("  DIO1 Pin: ", this->dio1_pin_);
  ESP_LOGCONFIG(TAG, "  Frequency: %" PRIu32 " Hz", this->frequency_);
  ESP_LOGCONFIG(TAG, "  Bandwidth: %" PRIu32 " Hz", BW_HZ[this->bandwidth_]);
  ESP_LOGCONFIG(TAG, "  PA Power: %" PRId8 " dBm", this->pa_power_);
  ESP_LOGCONFIG(TAG, "  PA Ramp: %" PRIu16 " us", RAMP[this->pa_ramp_]);
  if (this->modulation_ == PACKET_TYPE_GFSK) {
    ESP_LOGCONFIG(TAG, "  Modulation: %s", "FSK");
    ESP_LOGCONFIG(TAG, "  Deviation: %" PRIu32 " Hz", this->deviation_);
    if (this->shaping_ == GAUSSIAN_BT_0_3) {
      ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_0_3");
    } else if (this->shaping_ == GAUSSIAN_BT_0_5) {
      ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_0_5");
    } else if (this->shaping_ == GAUSSIAN_BT_0_7) {
      ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_0_7");
    } else if (this->shaping_ == GAUSSIAN_BT_1_0) {
      ESP_LOGCONFIG(TAG, "  Shaping: GAUSSIAN_BT_1_0");
    } else {
      ESP_LOGCONFIG(TAG, "  Shaping: NONE");
    }
    ESP_LOGCONFIG(TAG, "  Preamble Size: %" PRIu16, this->preamble_size_);
    ESP_LOGCONFIG(TAG, "  Preamble Detect: %" PRIu16, this->preamble_detect_);
    ESP_LOGCONFIG(TAG, "  Bitrate: %" PRIu32 "b/s", this->bitrate_);
  } else if (this->modulation_ == PACKET_TYPE_LORA) {
    ESP_LOGCONFIG(TAG, "  Modulation: %s", "LORA");
    ESP_LOGCONFIG(TAG, "  Spreading Factor: %" PRIu8, this->spreading_factor_);
    if (this->coding_rate_ == LORA_CR_4_5) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/5");
    } else if (this->coding_rate_ == LORA_CR_4_6) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/6");
    } else if (this->coding_rate_ == LORA_CR_4_7) {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/7");
    } else {
      ESP_LOGCONFIG(TAG, "  Coding Rate: 4/8");
    }
    ESP_LOGCONFIG(TAG, "  Preamble Size: %" PRIu16, this->preamble_size_);
  }
  if (!this->sync_value_.empty()) {
    ESP_LOGCONFIG(TAG, "  Sync Value: 0x%s", format_hex(this->sync_value_).c_str());
  }
  ESP_LOGCONFIG(TAG, "  Payload Length: %" PRIu32, this->payload_length_);
  ESP_LOGCONFIG(TAG, "  CRC Enable: %s", TRUEFALSE(this->crc_enable_));
  ESP_LOGCONFIG(TAG, "  Rx Start: %s", TRUEFALSE(this->rx_start_));
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Configuring SX126x failed");
  }
}

}  // namespace sx126x
}  // namespace esphome
