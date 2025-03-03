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
  for(int i = 0; i < packet.size(); i++) {
    packet[i] = this->transfer_byte(0x00);
  }
  // this->read_array(data, size);
  this->disable();
  this->wait_busy_();
  return status;
}

void SX126x::write_fifo_(uint8_t offset, const std::vector<uint8_t> &packet) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(RADIO_WRITE_BUFFER);
  this->transfer_byte(offset);
  for(int i = 0; i < packet.size(); i++) {
     this->transfer_byte(packet[i]);
  }
  this->disable();
  this->wait_busy_();
}

uint8_t SX126x::read_opcode_(uint8_t opcode, uint8_t *data, uint8_t size) {
  this->wait_busy_();
  this->enable();
  this->transfer_byte(opcode);
  uint8_t status = this->transfer_byte(0x00);
  for(int i = 0; i < size; i++) {
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
  for(int i = 0; i < size; i++) {
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
  for(int i = 0; i < size; i++) {
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
  for(int i = 0; i < size; i++) {
     this->transfer_byte(data[i]);
  }
  this->disable();
  this->wait_busy_();
}

uint8_t SX126x::read_register_(uint16_t reg) {
  uint8_t data;
  this->read_register_(reg, &data, 1);
  return data;
}

void SX126x::write_register_(uint16_t reg, uint8_t data) {
  this->write_register_(reg, &data, 1);
}

// void SX126x::read_fifo_(std::vector<uint8_t> &packet) {
//   this->enable();
//   RADIO_WRITE_BUFFER
//   this->write_byte(REG_FIFO & 0x7F);
//   this->read_array(packet.data(), packet.size());
//   this->disable();
// }

// void SX126x::write_fifo_(const std::vector<uint8_t> &packet) {
//   this->enable();
//   this->write_byte(REG_FIFO | 0x80);
//   this->write_array(packet.data(), packet.size());
//   this->disable();
// }

void SX126x::setup() {
  ESP_LOGCONFIG(TAG, "Setting up SX126x...");

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
  uint8_t status;
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

  buf[0] = 0x00;
  buf[1] = 0x00;
  this->write_opcode_(RADIO_CLR_ERROR, buf, 2);

  // config tcxo
  if (this->tcxo_voltage_ != TCXO_CTRL_NONE) {
    uint32_t delay = this->tcxo_delay_ >> 6;
    buf[0] = this->tcxo_voltage_;
    buf[1] = (delay >> 16) & 0xFF;
    buf[2] = (delay >> 8) & 0xFF;
    buf[3] = (delay >> 0) & 0xFF;
    this->write_opcode_(RADIO_SET_TCXOMODE, buf, 4);
  }

  // rf switch
  if (this->rf_switch_) {
    buf[0] = 0x01;
    this->write_opcode_(RADIO_SET_RFSWITCHMODE, buf, 1);
  }

  // buf[0] = 0x7F;
  // this->write_opcode_(RADIO_CALIBRATE, buf, 1);



    // buf[0] = 0x01;
    // this->write_opcode_(RADIO_SET_RFSWITCHMODE, buf, 1);




      // this->read_opcode_(RADIO_GET_STATUS, buf, 1);
      // ESP_LOGE(TAG, "status %02x", buf[0]);


  // buf[0] = MODE_STDBY_XOSC;
  // this->write_opcode_(RADIO_SET_STANDBY, buf, 1);



  // check silicon version to make sure hw is ok
  this->read_register_(REG_VERSION_STRING, (uint8_t*) this->version_, 16);
  if (strncmp(this->version_, "SX126", 5) != 0) {
    this->mark_failed();
    return;
  }




  // // setup buffer
  // buf[0] = 0;
  // buf[1] = 0;
  // this->write_opcode_(RADIO_SET_BUFFERBASEADDRESS, buf, 2);

  // setup packet type
  buf[0] = this->modulation_;
  this->write_opcode_(RADIO_SET_PACKETTYPE, buf, 1);

  // buf[0] = 0x01;
  // this->write_opcode_(RADIO_SET_REGULATORMODE, buf, 1);



  // if (this->frequency_ > 900000000) {
  //   buf[0] = 0xE1;
  //   buf[1] = 0xE9;
  // } else if (this->frequency_ > 850000000) {
  //   buf[0] = 0xD7;
  //   buf[1] = 0xD8;
  // } else if (this->frequency_ > 770000000) {
  //   buf[0] = 0xC1;
  //   buf[1] = 0xC5;
  // } else if (this->frequency_ > 460000000) {
  //   buf[0] = 0x75;
  //   buf[1] = 0x81;
  // } else if (this->frequency_ > 425000000) {
  //   buf[0] = 0x6B;
  //   buf[1] = 0x6F;
  // }
  // this->write_opcode_(RADIO_CALIBRATEIMAGE, buf, 2);

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
    this->write_register_(REG_OCP, 0x18); // max 80 mA
  } else {
    uint8_t cfg[4] = {0x04, 0x07, 0x00, 0x01};
    this->write_opcode_(RADIO_SET_PACONFIG, cfg, 4);
    pa_power = std::max(pa_power, (int8_t) -3);
    pa_power = std::min(pa_power, (int8_t) 22);
    this->write_register_(REG_OCP, 0x38); // max 140 mA
  }
  buf[0] = pa_power;
  buf[1] = this->pa_ramp_;
  this->write_opcode_(RADIO_SET_TXPARAMS, buf, 2);

  // configure modem
  if (this->modulation_ == PACKET_TYPE_LORA) {
    // set modulation params
    float duration = 1000.0f * std::pow(2, this->spreading_factor_) / BW_HZ[this->bandwidth_];
    buf[0] = this->spreading_factor_;
    buf[1] = BW_LORA[this->bandwidth_];
    buf[2] = this->coding_rate_;
    buf[3] = (duration > 16.38f) ? 0x01 : 0x00;
    this->write_opcode_(RADIO_SET_MODULATIONPARAMS, buf, 4);

    this->set_packet_params_(this->payload_length_);

    // write sync word
    this->write_register_(REG_LR_SYNCWORD + 0, 0x14);
    this->write_register_(REG_LR_SYNCWORD + 1, 0x24);
  }

  // switch to rx or standby
  if (this->rx_start_) {
    this->set_mode_rx();
  } else {
    this->set_mode_standby();
  }
}

void SX126x::set_packet_params_(uint8_t payload_length) {
    uint8_t buf[6];
    buf[0] = (this->preamble_size_ >> 8) & 0xFF;
    buf[1] = (this->preamble_size_ >> 0) & 0xFF;
    buf[2] = (this->payload_length_ > 0) ? 0x01 : 0x00;
    buf[3] = payload_length;
    buf[4] = (this->crc_enable_) ? 0x01 : 0x00;
    buf[5] = 0x00;
    this->write_opcode_(RADIO_SET_PACKETPARAMS, buf, 6);
}

void SX126x::transmit_packet(const std::vector<uint8_t> &packet) {
  if (this->payload_length_ > 0 && this->payload_length_ != packet.size()) {
    ESP_LOGE(TAG, "Packet size does not match payload length");
    return;
  }
  if (packet.empty() || packet.size() > 256) {
    ESP_LOGE(TAG, "Packet size out of range");
    return;
  }
  this->set_mode_standby();
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
    this->set_mode_standby();
  }
}

void SX126x::loop() {
  if (this->dio1_pin_->digital_read()) {
    uint16_t status;
    uint8_t buf[3];
    uint8_t rssi;
    int8_t snr;
    this->read_opcode_(RADIO_GET_IRQSTATUS, buf, 2);
    status = (buf[0] << 8) | buf[1];
    if ((status & IRQ_RX_DONE) == IRQ_RX_DONE) {
      if ((status & IRQ_CRC_ERROR) != IRQ_CRC_ERROR) {
        this->read_opcode_(RADIO_GET_PACKETSTATUS, buf, 3);
        if (this->modulation_ == PACKET_TYPE_LORA) {
          rssi  = buf[0];
          snr  = buf[1];
        } else {
          rssi  = buf[2];
          snr = 0;
        }
        this->read_opcode_(RADIO_GET_RXBUFFERSTATUS, buf, 2);
        std::vector<uint8_t> packet(buf[0]);
        this->read_fifo_(buf[1], packet);
        this->packet_trigger_->trigger(packet, (float) rssi / -2.0f, (float) snr / 4.0f);
      }
    }
    buf[0] = (status >> 8) & 0xFF;
    buf[1] = (status >> 0) & 0xFF;
    this->write_opcode_(RADIO_CLR_IRQSTATUS, buf, 2);
  }
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
  uint8_t buf[8];

  // configure irq params
  uint16_t irq =  IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR;
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
  uint16_t irq =  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
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

void SX126x::set_mode_standby() {
  uint8_t buf[1];
  buf[0] = MODE_STDBY_RC;
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
