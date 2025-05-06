#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sx126x/sx126x.h"
#include "esphome/components/packet_transport/packet_transport.h"
#include <vector>

namespace esphome {
namespace sx126x {

class SX126xTransport : public packet_transport::PacketTransport, public Parented<SX126x>, public SX126xListener {
 public:
  void update() override;
  void on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

 protected:
  void send_packet(std::vector<uint8_t> &buf) const override;
  size_t get_max_packet_size() override { return this->parent_->get_max_packet_size(); }
  bool should_send() { return true; }
};

}  // namespace sx126x
}  // namespace esphome
