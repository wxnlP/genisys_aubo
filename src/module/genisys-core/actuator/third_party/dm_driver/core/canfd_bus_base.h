#pragma once

#include <cstdint>

class CanFdBusBase {
 public:
  CanFdBusBase() = default;
  virtual ~CanFdBusBase() = default;

  virtual int Send(uint32_t id, const uint8_t* data, uint8_t len) {
    (void)id;
    (void)data;
    (void)len;
    return -1;
  }
  virtual int Receive(uint32_t id, uint8_t* data) {
    (void)id;
    (void)data;
    return -1;
  }
};