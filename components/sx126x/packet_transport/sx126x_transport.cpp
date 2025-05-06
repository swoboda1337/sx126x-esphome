#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/network/util.h"
#include "sx126x_transport.h"

namespace esphome {
namespace sx126x {

static const char *const TAG = "sx126x_transport";

void SX126xTransport::update() {
  PacketTransport::update();
  this->updated_ = true;
  this->resend_data_ = true;
}

void SX126xTransport::send_packet(std::vector<uint8_t> &buf) const { this->parent_->transmit_packet(buf); }

void SX126xTransport::on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) {
  std::vector<uint8_t> temp = packet;
  this->process_(temp);
}

}  // namespace sx126x
}  // namespace esphome
