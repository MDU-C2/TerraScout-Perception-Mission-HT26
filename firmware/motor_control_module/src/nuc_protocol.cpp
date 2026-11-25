#include "nuc_protocol.hpp"

uint8_t nuc_checksum(const uint8_t * packet)
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < config::NUC_PACKET_SIZE - 1; ++i) {
    checksum ^= packet[i];
  }
  return checksum;
}
